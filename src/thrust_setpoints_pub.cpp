#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <Eigen/Dense>
#include <cmath>

/*
 * Thruster allocation for Iris.
 *
 * tau = B * f  ->  f = B^+ * tau
 *   tau = [Fx, Fy, Fz, Mx, My, Mz] body wrench (NED body frame: x fwd, y stbd, z down)
 *   f   = force of each thruster [N], in the order the actuators are defined in
 *         iris_full.scn (Stonefish maps setpoint index -> definition order):
 *         0 Surge1  1 Surge2  2 Surge3  3 Surge4
 *         4 HeaveLeft  5 HeaveRight  6 SwayFront  7 SwayBack
 *
 * Column i of B = [d_i ; (r_i - cg) x d_i], with r_i the thruster position and d_i its
 * thrust axis, both taken from iris_full.scn.
 *
 * Input (/iris/controller/cmd_vel):
 *   linear.x/y : surge/sway force  (x surge_sway_gain N)
 *   linear.z   : heave force, +down (x heave_gain N)
 *   angular.x/y/z : roll/pitch/yaw angular acceleration [rad/s^2] (x inertia)
 */
class ThrustSetpointsPublisher : public rclcpp::Node
{
public:
    static constexpr int N_THR = 8;

    ThrustSetpointsPublisher() : Node("pub_setpoints")
    {
        // Thruster model, must match iris_full.scn (Stonefish FDThrust, zero-order rotor).
        // Calibrated to Blue Robotics T200 @16 V: 5.25 kgf forward / 4.1 kgf reverse at ~3500 rpm.
        accel_        = declare_parameter("surge_sway_gain", 4.0);   // cmd_vel linear.x/y -> force [N]
        heave_gain_   = declare_parameter("heave_gain", 2.0);        // cmd_vel linear.z -> force [N]
        buoyancy_off_ = declare_parameter("buoyancy_offset", 0.0);   // constant extra Fz [N]
        const double rho       = declare_parameter("water_density", 1000.0);
        const double diameter  = declare_parameter("propeller_diameter", 0.079);
        const double kt_fwd    = declare_parameter("thrust_coeff_forward", 0.3885);
        const double kt_rev    = declare_parameter("thrust_coeff_reverse", 0.3034);
        const double max_omega = declare_parameter("max_setpoint", 366.52); // rad/s at setpoint 1.0

        // Static thrust at full setpoint: T = rho * D^4 * kt * n^2, n = omega / 2pi
        const double n_max = max_omega / (2.0 * M_PI);
        const double base = rho * std::pow(diameter, 4) * n_max * n_max;
        thrust_fwd_ = base * kt_fwd;   // force along +thruster axis [N]
        thrust_rev_ = base * kt_rev;   // force along -thruster axis [N]

        // All thrusters have inverted_setpoint="true" and right-handed propellers:
        // a positive setpoint pushes along the NEGATIVE thruster axis -> sign -1.
        auto signs = declare_parameter("thruster_signs",
            std::vector<double>(N_THR, -1.0));
        if (signs.size() != N_THR) {
            RCLCPP_WARN(get_logger(), "thruster_signs must have %d entries, using all -1", N_THR);
            signs.assign(N_THR, -1.0);
        }
        for (int i = 0; i < N_THR; ++i) signs_(i) = signs[i];

        // Thruster geometry from iris_full.scn (body origin, +x bow): position r_i and thrust
        // axis d_i. Surge thrusters sit at the stern (-x) and push towards the bow (+x).
        const double pos[N_THR][3] = {
            {-0.43, 0.1653, -0.089}, {-0.43, -0.1653, -0.089}, {-0.43, -0.1653, 0.105}, {-0.43, 0.1653, 0.105},
            {0.0, -0.18, 0.01}, {0.0, 0.18, 0.01}, {-0.217, 0.0, 0.065}, {0.15, 0.0, 0.065}};
        const double dir[N_THR][3] = {
            {1, 0, 0}, {1, 0, 0}, {1, 0, 0}, {1, 0, 0},
            {0, 0, 1}, {0, 0, 1}, {0, 1, 0}, {0, -1, 0}};

        // Moments are taken about the centre of gravity. Values match iris_full.scn after the
        // mass-model fix (20 kg, explicit part masses): CG from the part masses (confirmed in sim
        // with thrust pulses), inertia = Stonefish's principal inertia incl. hull added inertia.
        const auto cg = declare_parameter("cg", std::vector<double>{-0.032, 0.0, -0.020});
        const auto inertia = declare_parameter("inertia", std::vector<double>{0.23, 0.88, 1.22});
        I_ = Eigen::Vector3d(inertia[0], inertia[1], inertia[2]);

        // Allocation matrix (rows: Fx Fy Fz Mx My Mz), column i = [d_i ; (r_i - cg) x d_i]
        Eigen::Matrix<double, 6, N_THR> B;
        for (int i = 0; i < N_THR; ++i) {
            const Eigen::Vector3d r(pos[i][0] - cg[0], pos[i][1] - cg[1], pos[i][2] - cg[2]);
            const Eigen::Vector3d d(dir[i][0], dir[i][1], dir[i][2]);
            B.col(i) << d, r.cross(d);
        }
        Binv_ = B.completeOrthogonalDecomposition().pseudoInverse();

        sub_pilot_ = create_subscription<geometry_msgs::msg::Twist>(
            "/iris/controller/cmd_vel", 10,
            std::bind(&ThrustSetpointsPublisher::pilotCallback, this, std::placeholders::_1));
        pub_thrusters_ = create_publisher<std_msgs::msg::Float64MultiArray>(
            "/iris/controller/thruster_setpoints", 10);
        // Allocation runs as soon as a command arrives (no extra timer latency in the
        // attitude loop); this timer only zeroes the thrusters if commands stop.
        last_cmd_ = now();
        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ThrustSetpointsPublisher::watchdog, this));

        RCLCPP_INFO(get_logger(), "Inertia = (%.1f, %.1f, %.1f), thruster limits: %.1f N fwd / %.1f N rev",
            I_(0), I_(1), I_(2), thrust_fwd_, thrust_rev_);
    }

private:
    void pilotCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        descent_cmd_ = msg->linear.z;
        fwd_cmd_ = msg->linear.x;
        lat_cmd_ = msg->linear.y;
        roll_cmd_ = msg->angular.x;
        pitch_cmd_ = msg->angular.y;
        yaw_cmd_ = msg->angular.z;
        last_cmd_ = now();
        controlLoop();
    }

    void watchdog()
    {
        if ((now() - last_cmd_).seconds() < 0.5) return;
        descent_cmd_ = fwd_cmd_ = lat_cmd_ = roll_cmd_ = pitch_cmd_ = yaw_cmd_ = 0.0;
        controlLoop();
    }

    // Largest a in [0,1] such that base + a*add stays within every thruster's limits
    // (T200 is weaker in reverse, so the limit depends on the direction).
    double maxScale(const Eigen::Matrix<double, N_THR, 1> &base,
                    const Eigen::Matrix<double, N_THR, 1> &add) const
    {
        double a = 1.0;
        for (int i = 0; i < N_THR; ++i) {
            if (add(i) > 1e-9)       a = std::min(a, (thrust_fwd_ - base(i)) / add(i));
            else if (add(i) < -1e-9) a = std::min(a, (-thrust_rev_ - base(i)) / add(i));
        }
        return std::max(a, 0.0);
    }

    void controlLoop()
    {
        // Desired body wrench
        Eigen::Matrix<double, 6, 1> tau;
        tau << accel_ * fwd_cmd_,
               accel_ * lat_cmd_,
               heave_gain_ * descent_cmd_ + buoyancy_off_,
               I_(0) * roll_cmd_,   // angular.x/y/z are angular accelerations [rad/s^2]
               I_(1) * pitch_cmd_,
               I_(2) * yaw_cmd_;

        // Prioritised allocation. Level 1: heave + roll/pitch (keeps the vehicle level and
        // at depth). Level 2: surge, sway, yaw get whatever thruster capacity is left.
        // Each level is scaled as a whole, which keeps the direction of its wrench.
        Eigen::Matrix<double, 6, 1> tau1 = Eigen::Matrix<double, 6, 1>::Zero(), tau2 = tau1;
        tau1(2) = tau(2); tau1(3) = tau(3); tau1(4) = tau(4);
        tau2(0) = tau(0); tau2(1) = tau(1); tau2(5) = tau(5);
        Eigen::Matrix<double, N_THR, 1> f1 = Binv_ * tau1, f2 = Binv_ * tau2;
        f1 *= maxScale(Eigen::Matrix<double, N_THR, 1>::Zero(), f1);
        Eigen::Matrix<double, N_THR, 1> f = f1 + maxScale(f1, f2) * f2;

        // Force -> normalized setpoint. Thrust ~ omega^2 and omega = setpoint * max_setpoint,
        // so setpoint = sign * sqrt(|f| / full_scale_thrust_in_that_direction).
        std::vector<double> setpoints(N_THR);
        for (int i = 0; i < N_THR; ++i) {
            const double lim = f(i) >= 0.0 ? thrust_fwd_ : thrust_rev_;
            const double s = std::sqrt(std::abs(f(i)) / lim);
            setpoints[i] = signs_(i) * std::copysign(std::min(s, 1.0), f(i));
        }

        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data = setpoints;
        pub_thrusters_->publish(msg);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "thrust [N]: %.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
            f(0), f(1), f(2), f(3), f(4), f(5), f(6), f(7));
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "setpoints: %.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
            setpoints[0], setpoints[1], setpoints[2], setpoints[3],
            setpoints[4], setpoints[5], setpoints[6], setpoints[7]);
    }

    double accel_, heave_gain_, buoyancy_off_, thrust_fwd_, thrust_rev_;
    Eigen::Vector3d I_;
    rclcpp::Time last_cmd_;
    double roll_cmd_ = 0.0, pitch_cmd_ = 0.0, yaw_cmd_ = 0.0;
    double descent_cmd_ = 0.0, fwd_cmd_ = 0.0, lat_cmd_ = 0.0;
    Eigen::Matrix<double, N_THR, 6> Binv_;
    Eigen::Matrix<double, N_THR, 1> signs_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_pilot_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_thrusters_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ThrustSetpointsPublisher>());
    rclcpp::shutdown();
    return 0;
}
