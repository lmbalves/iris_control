/**
 * @file tag_range_estimator.cpp
 * @brief Range-only estimate of the vehicle->tag vector (level NED) from USBL ranges taken
 *        while loitering. Only |marker.position| is used, never its direction.
 *        Publishes /iris/tag_rel (20 Hz, dead-reckoned) and /iris/tag_converged.
 *        Until converged, publishes a virtual tag at the start point at the current depth,
 *        so the pilot orbits that while pings accumulate.
 *
 * Depth handling: on a depth-flat loiter the sign of dz is ambiguous (mirror solution), so a
 * "tag is below the sensor" prior is applied, but ONLY while the ping window has no vertical
 * spread. Once the vehicle has moved vertically dz is observable and the fit decides alone.
 * If the tag depth is known (tank floor, seabed), set `tag_depth` and the ambiguity is gone.
 */
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "stonefish_ros2/msg/dvl.hpp"
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <vector>

class TagRangeEstimator : public rclcpp::Node
{
public:
    TagRangeEstimator() : Node("tag_range_estimator")
    {
        sigma_    = declare_parameter("range_sigma", 0.1);      // [m]
        window_   = declare_parameter("window", 40);            // pings kept
        min_n_    = declare_parameter("min_pings", 12);
        conv_std_ = declare_parameter("converged_std", 0.3);    // horizontal 1-sigma [m]
        min_dz_   = declare_parameter("min_dz", 0.5);           // "tag below sensor" floor, depth-flat window only [m]
        depth_excitation_ = declare_parameter("depth_excitation", 1.0); // window depth spread needed to trust dz [m]
        tag_depth_ = declare_parameter("tag_depth", std::numeric_limits<double>::quiet_NaN()); // known tag depth, NED down [m]
        const auto usbl = declare_parameter("usbl_offset", std::vector<double>{0.0, 0.0, 0.5});
        const auto rpy  = declare_parameter("dvl_rpy", std::vector<double>{3.1416, 0.0, -0.785});
        usbl_off_ = tf2::Vector3(usbl[0], usbl[1], usbl[2]);
        dvl_mount_.setRPY(rpy[0], rpy[1], rpy[2]);

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("/iris/navigator/imu", 10,
            [this](sensor_msgs::msg::Imu::SharedPtr m) {
                R_ = tf2::Matrix3x3(tf2::Quaternion(m->orientation.x, m->orientation.y,
                                                    m->orientation.z, m->orientation.w));
                have_imu_ = true; });
        dvl_sub_ = create_subscription<stonefish_ros2::msg::DVL>("/iris/navigator/dvl", 10,
            [this](stonefish_ros2::msg::DVL::SharedPtr m) {
                v_body_ = dvl_mount_ * tf2::Vector3(m->velocity.x, m->velocity.y, m->velocity.z); });
        usbl_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>("/iris/navigator/usbl", 10,
            [this](visualization_msgs::msg::MarkerArray::SharedPtr m) { onPing(*m); });

        tag_pub_  = create_publisher<geometry_msgs::msg::PointStamped>("/iris/tag_rel", 10);
        conv_pub_ = create_publisher<std_msgs::msg::Bool>("/iris/tag_converged", 10);
        timer_ = create_wall_timer(std::chrono::milliseconds(50), [this] { step(); });
    }

private:
    using V3 = Eigen::Vector3d;
    struct Ping { V3 s; double r; };

    static V3 eig(const tf2::Vector3 &v) { return V3(v.x(), v.y(), v.z()); }

    void onPing(const visualization_msgs::msg::MarkerArray &m)
    {
        if (m.markers.empty() || !have_imu_) return;
        const auto &p = m.markers[0].pose.position;
        const double r = std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);   // range only
        const V3 s = x_ + eig(R_ * usbl_off_);
        pings_.push_back({s, r});
        if ((int)pings_.size() > window_) pings_.pop_front();
        solve();
    }

    // Depth spread of the sensor positions in the current window
    double depthSpread() const
    {
        double zmin = pings_.front().s.z(), zmax = zmin;
        for (const auto &pg : pings_) {
            zmin = std::min(zmin, pg.s.z());
            zmax = std::max(zmax, pg.s.z());
        }
        return zmax - zmin;
    }

    void initLinear()
    {
        // r^2 - |s_h|^2 = -2 s_h.p_h + c,  c = |p_h|^2 + dz^2   (constant-depth safe)
        const int n = pings_.size();
        Eigen::MatrixXd A(n, 3); Eigen::VectorXd b(n);
        double zbar = 0;
        for (int i = 0; i < n; ++i) {
            const V3 &s = pings_[i].s; zbar += s.z() / n;
            A.row(i) << -2 * s.x(), -2 * s.y(), 1.0;
            b(i) = pings_[i].r * pings_[i].r - s.x()*s.x() - s.y()*s.y();
        }
        const V3 sol = A.colPivHouseholderQr().solve(b);
        const double dz2 = sol.z() - (sol.x()*sol.x() + sol.y()*sol.y());
        p_ = V3(sol.x(), sol.y(), zbar + std::max(std::sqrt(std::max(dz2, 0.0)), min_dz_)); // tag below
        if (std::isfinite(tag_depth_)) p_.z() = tag_depth_;
    }

    void solve()
    {
        if ((int)pings_.size() < min_n_) return;
        if (!have_est_) { initLinear(); have_est_ = true; }
        double lambda = 1e-3;
        Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
        for (int it = 0; it < 15; ++it) {
            H.setZero(); V3 g = V3::Zero();
            for (const auto &pg : pings_) {
                const V3 d = p_ - pg.s; const double rho = std::max(d.norm(), 1e-3);
                const double res = rho - pg.r;
                const double w = (std::abs(res) < 3 * sigma_) ? 1.0 : 3 * sigma_ / std::abs(res); // Huber
                const V3 J = d / rho;
                H += w * J * J.transpose(); g += w * J * res;
            }
            const V3 dp = -(H + lambda * Eigen::Matrix3d::Identity()).ldlt().solve(g);
            p_ += dp;
            if (dp.norm() < 1e-4) break;
        }

        // Depth: known tag depth wins; otherwise the "below the sensor" prior applies only
        // while the window is depth-flat (mirror ambiguity). With vertical spread dz is observable.
        if (std::isfinite(tag_depth_)) {
            p_.z() = tag_depth_;
        } else if (depthSpread() < depth_excitation_) {
            const double zs = pings_.back().s.z();
            if (p_.z() < zs + min_dz_) p_.z() = zs + min_dz_;
        }
        cov_ = (H + 1e-6 * Eigen::Matrix3d::Identity()).inverse() * sigma_ * sigma_;

        // Latch with hysteresis: converge below conv_std_, drop only above 2x
        const double std_h = std::sqrt(std::max(cov_(0, 0) + cov_(1, 1), 0.0));
        if (!converged_ && std_h < conv_std_)           converged_ = true;
        else if (converged_ && std_h > 2.0 * conv_std_) converged_ = false;
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
            "tag est N%+.2f E%+.2f D%+.2f  std_h=%.2f  n=%zu  dspread=%.2f  %s",
            p_.x(), p_.y(), p_.z(), std_h, pings_.size(), depthSpread(),
            converged_ ? "CONVERGED" : "searching");
    }

    void step()
    {
        if (!have_imu_) return;
        const double dt = 0.05;
        x_ += eig(R_ * v_body_) * dt;                    // dead-reckoned position, odom NED
        if (!origin_set_) { search_center_ = x_; origin_set_ = true; }

        // converged: real tag; otherwise a virtual tag at the start point, at current depth
        V3 rel;
        if (converged_) rel = p_ - x_;
        else            rel = V3(search_center_.x() - x_.x(), search_center_.y() - x_.y(), 0.0);

        geometry_msgs::msg::PointStamped out;
        out.header.stamp = now(); out.header.frame_id = "ned_level";
        out.point.x = rel.x(); out.point.y = rel.y(); out.point.z = rel.z();
        tag_pub_->publish(out);
        std_msgs::msg::Bool c; c.data = converged_; conv_pub_->publish(c);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<stonefish_ros2::msg::DVL>::SharedPtr dvl_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr usbl_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr tag_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr conv_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    tf2::Matrix3x3 R_{tf2::Quaternion::getIdentity()}, dvl_mount_;
    tf2::Vector3 v_body_{0, 0, 0}, usbl_off_;
    V3 x_ = V3::Zero(), p_ = V3::Zero(), search_center_ = V3::Zero();
    Eigen::Matrix3d cov_ = Eigen::Matrix3d::Identity() * 1e3;
    std::deque<Ping> pings_;
    bool have_imu_ = false, have_est_ = false, converged_ = false, origin_set_ = false;
    double sigma_, conv_std_, min_dz_, depth_excitation_, tag_depth_; int window_, min_n_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TagRangeEstimator>());
    rclcpp::shutdown();
    return 0;
}