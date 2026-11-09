/**
 * @file estimated_acoustic_pilot.cpp
 * @author Luis Alves (lmbalves@gmail.com)
 * @brief Acoustic guidance driven by an external tag-position estimate: descend towards the
 *        tag while spiralling in, then circle it at a fixed radius, using a Lyapunov guidance
 *        vector field for heading. Pitch follows the commanded flight-path angle; surge speed
 *        is the along-path speed so the heave thrusters only trim.
 * @version 0.5
 *
 * Difference to acoustic_pilot: this node does NOT read the USBL and does NOT dead-reckon the
 * tag. The tag_range_estimator node (range-only fit while loitering) publishes the
 * vehicle->tag vector in level NED on /iris/tag_rel at 20 Hz, already dead-reckoned. While the
 * estimate has not converged the estimator publishes a virtual tag at the start point, so this
 * node simply orbits that until the real one appears.
 *
 * Frames: everything is NED. IMU gives the attitude, DVL the body velocity.
 *
 * Output (/iris/controller/cmd_vel, consumed by thrust_setpoints_pub):
 *   linear.x / linear.y : surge / sway force command   (x4 N in thrust_setpoints_pub)
 *   linear.z            : heave force command, +down   (x2 N)
 *   angular.x/y/z       : roll/pitch/yaw angular acceleration [rad/s^2] (x inertia N*m)
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "stonefish_ros2/msg/dvl.hpp"

#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <algorithm>
#include <cmath>
#include <vector>

namespace
{
double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }
double clamp(double v, double lim) { return std::clamp(v, -lim, lim); }
}

class EstimatedAcousticPilot : public rclcpp::Node
{
public:
    EstimatedAcousticPilot() : Node("estimated_acoustic_pilot")
    {
        // Guidance
        radius_      = declare_parameter("radius", 3.0);          // circle radius around the tag [m]
        k_           = declare_parameter("k", 1.0);               // shape exponent of the vector field
        standoff_    = declare_parameter("depth_standoff", 0.0);  // circle this far above the tag [m]
        cruise_      = declare_parameter("cruise_speed", 0.3);    // surge speed setpoint [m/s]
        tag_timeout_ = declare_parameter("tag_timeout", 0.5);     // hold still if no tag_rel for this long [s]
        // Heading / yaw-rate loops
        k_psi_       = declare_parameter("k_psi", 0.6);           // heading error -> yaw rate [1/s]
        r_max_       = declare_parameter("max_yaw_rate", 0.2);    // [rad/s]; higher spins couple into roll/pitch (Izz >> Ixx,Iyy)
        k_rate_      = declare_parameter("k_yaw_rate", 5.0);      // yaw-rate error -> yaw accel [1/s] (30 chattered at 20 Hz)
        ki_rate_     = declare_parameter("ki_yaw_rate", 5.0);     // removes the turn-rate lag that widened the circle
        // Roll / pitch attitude loop (PID, output in rad/s^2). Pitch follows the commanded
        // flight-path angle (nose down while descending), roll is held at zero. The integral
        // carries the standing moments: surge thrust lines above/below the CG and the
        // hydrostatic righting moment when pitched (CB ~4 cm above CG -> ~5 N*m at 40 deg).
        k_att_       = declare_parameter("k_attitude", 4.0);
        kd_att_      = declare_parameter("kd_attitude", 4.0);
        ki_att_      = declare_parameter("ki_attitude", 3.0);
        max_pitch_   = declare_parameter("max_pitch", 0.8);       // [rad] limit on the pitch setpoint (45 deg at cruise = max descent rate)
        // Surge / sway
        k_u_         = declare_parameter("k_surge", 20.0);
        ki_u_        = declare_parameter("ki_surge", 5.0);
        k_v_         = declare_parameter("k_sway", 20.0);
        // Depth
        k_z_         = declare_parameter("k_depth", 0.3);         // depth error -> descent rate [1/s]
        w_max_       = declare_parameter("max_descent_rate", 0.3);// [m/s]
        k_w_         = declare_parameter("k_heave", 40.0);
        ki_w_        = declare_parameter("ki_heave", 10.0);
        cmd_lin_max_ = declare_parameter("max_linear_cmd", 25.0); // cmd units (x4 N surge/sway, x2 N heave)
        // Sensor mounting (iris_full.scn)
        const auto dvl_rpy = declare_parameter("dvl_rpy", std::vector<double>{3.1416, 0.0, -0.785});
        dvl_mount_.setRPY(dvl_rpy[0], dvl_rpy[1], dvl_rpy[2]);

        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/iris/controller/cmd_vel", 10);
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/iris/navigator/imu", 10,
            std::bind(&EstimatedAcousticPilot::imuCallback, this, std::placeholders::_1));
        dvl_sub_ = create_subscription<stonefish_ros2::msg::DVL>(
            "/iris/navigator/dvl", 10,
            std::bind(&EstimatedAcousticPilot::dvlCallback, this, std::placeholders::_1));
        tag_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
            "/iris/tag_rel", 10,
            std::bind(&EstimatedAcousticPilot::tagCallback, this, std::placeholders::_1));

        timer_ = create_wall_timer(std::chrono::milliseconds(50),
            std::bind(&EstimatedAcousticPilot::controlLoop, this));
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        tf2::Quaternion q(msg->orientation.x, msg->orientation.y,
                          msg->orientation.z, msg->orientation.w);
        R_ = tf2::Matrix3x3(q);
        R_.getRPY(roll_, pitch_, yaw_);
        roll_rate_ = msg->angular_velocity.x;
        pitch_rate_ = msg->angular_velocity.y;
        // Heading rate (Euler), not body z-rate: they differ when the vehicle is tilted
        yaw_rate_ = (pitch_rate_ * std::sin(roll_) + msg->angular_velocity.z * std::cos(roll_))
                    / std::max(std::cos(pitch_), 0.2);
        have_imu_ = true;
    }

    void dvlCallback(const stonefish_ros2::msg::DVL::SharedPtr msg)
    {
        // DVL frame -> body frame
        v_body_ = dvl_mount_ * tf2::Vector3(msg->velocity.x, msg->velocity.y, msg->velocity.z);
    }

    void tagCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        // Vehicle -> tag, level NED, already dead-reckoned by the estimator
        tag_rel_ = tf2::Vector3(msg->point.x, msg->point.y, msg->point.z);
        last_tag_rx_ = now();
        have_tag_ = true;
    }

    void controlLoop()
    {
        const double dt = 0.05;
        geometry_msgs::msg::Twist cmd;
        const bool tag_fresh = have_tag_ && (now() - last_tag_rx_).seconds() < tag_timeout_;
        if (!tag_fresh || !have_imu_) {      // nothing to steer to (yet / any more): hold still
            cmd_vel_pub_->publish(cmd);
            return;
        }

        const tf2::Vector3 v_world = R_ * v_body_;

        // ---------------- Horizontal guidance (Lyapunov vector field) ----------------
        const double range = std::max(std::hypot(tag_rel_.x(), tag_rel_.y()), 0.5);
        const double beta  = std::atan2(-tag_rel_.y(), -tag_rel_.x());  // bearing tag -> vehicle
        const double s     = std::clamp((range - radius_) / radius_, -1.0, 1.0);
        const double shape = std::copysign(std::pow(std::abs(s), k_), s);
        // On the circle: tangent (beta - pi/2). Far outside: 30 deg off a direct approach.
        // Inside: turned outwards. Continuous in range (saturates at 2R).
        const double psi_d = wrap(beta - M_PI_2 - (M_PI / 3.0) * shape);

        // Feed-forward: d(psi_d)/dt along the current motion
        const double V    = std::hypot(v_world.x(), v_world.y());
        const double chi  = std::atan2(v_world.y(), v_world.x());          // course over ground
        const double beta_dot  = (V / range) * std::sin(chi - beta);
        const double range_dot = V * std::cos(chi - beta);
        const double dshape_dr = (std::abs(s) < 1.0)
            ? (k_ / radius_) * std::pow(std::max(std::abs(s), 1e-3), k_ - 1.0) : 0.0;
        const double psi_d_dot = beta_dot - (M_PI / 3.0) * dshape_dr * range_dot;

        // Course -> yaw rate -> yaw acceleration. The field gives a desired direction of
        // travel, so steer the course: subtract the measured sideslip from the heading target
        // (Iris crabs outward in turns; steering heading alone settled ~1 m outside the circle).
        const double slip   = (V > 0.1) ? wrap(chi - yaw_) : 0.0;
        const double e_psi  = wrap(psi_d - slip - yaw_);
        const double r_cmd  = clamp(psi_d_dot + k_psi_ * e_psi, r_max_);
        const double e_r = r_cmd - yaw_rate_;
        integ_r_ = clamp(integ_r_ + e_r * dt, 0.5);
        cmd.angular.z = k_rate_ * e_r + ki_rate_ * integ_r_;

        // ---------------- Depth: go to tag depth - standoff ----------------
        const double e_z = tag_rel_.z() - standoff_;                // +: target is below
        const double w_d = clamp(k_z_ * e_z, w_max_);
        const double e_w = w_d - v_world.z();
        integ_w_ = clamp(integ_w_ + e_w * dt, 1.0);
        cmd.linear.z = clamp(k_w_ * e_w + ki_w_ * integ_w_, cmd_lin_max_);

        // ---------------- Attitude ----------------
        // Pitch the nose along the commanded path (NED: nose down is negative pitch), so the
        // vehicle descends inclined and levels out as it reaches the tag depth. Roll stays zero.
        const double pitch_d = clamp(-std::atan2(w_d, cruise_), max_pitch_);
        const double e_roll  = -roll_;
        const double e_pitch = pitch_d - pitch_;
        integ_roll_  = clamp(integ_roll_  + e_roll  * dt, 3.0);   // ki*3 = 9 rad/s^2 (~8 N*m) of standing moment
        integ_pitch_ = clamp(integ_pitch_ + e_pitch * dt, 3.0);
        cmd.angular.x = k_att_ * e_roll  - kd_att_ * roll_rate_  + ki_att_ * integ_roll_;
        cmd.angular.y = k_att_ * e_pitch - kd_att_ * pitch_rate_ + ki_att_ * integ_pitch_;

        // Surge speed: the along-path speed, so that with the nose on the path angle surge alone
        // gives `cruise` horizontally and `w_d` vertically (the heave loop then only trims).
        // Slowed down while the heading error is large. Sway is damped to zero.
        const double u_d = std::hypot(cruise_, w_d) * std::max(0.0, std::cos(e_psi));
        const double e_u = u_d - v_body_.x();
        integ_u_ = clamp(integ_u_ + e_u * dt, 5.0);   // ki_surge*5 = 25 cmd (100 N): enough to hold cruise against drag
        cmd.linear.x = clamp(k_u_ * e_u + ki_u_ * integ_u_, cmd_lin_max_);
        cmd.linear.y = clamp(-k_v_ * v_body_.y(), cmd_lin_max_);

        cmd_vel_pub_->publish(cmd);

        RCLCPP_DEBUG(get_logger(), "r=%.2f beta=%+.2f psi_d=%+.2f yaw=%+.2f r_cmd=%+.2f e_z=%+.2f pitch_d=%+.2f pitch=%+.2f",
            range, beta, psi_d, yaw_, r_cmd, e_z, pitch_d, pitch_);
    }

    // ROS
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<stonefish_ros2::msg::DVL>::SharedPtr dvl_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr tag_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_tag_rx_{0, 0, RCL_ROS_TIME};

    // State
    tf2::Matrix3x3 R_{tf2::Quaternion::getIdentity()}, dvl_mount_;
    tf2::Vector3 v_body_{0, 0, 0}, tag_rel_{0, 0, 0};
    double roll_ = 0.0, pitch_ = 0.0, yaw_ = 0.0;
    double roll_rate_ = 0.0, pitch_rate_ = 0.0, yaw_rate_ = 0.0, integ_u_ = 0.0, integ_w_ = 0.0, integ_r_ = 0.0;
    double integ_roll_ = 0.0, integ_pitch_ = 0.0;
    bool have_imu_ = false, have_tag_ = false;

    // Parameters
    double radius_, k_, standoff_, cruise_, tag_timeout_, k_psi_, r_max_, k_rate_, ki_rate_, k_att_, kd_att_, ki_att_, max_pitch_;
    double k_u_, ki_u_, k_v_, k_z_, w_max_, k_w_, ki_w_, cmd_lin_max_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EstimatedAcousticPilot>());
    rclcpp::shutdown();
    return 0;
}