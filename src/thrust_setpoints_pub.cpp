#include "rclcpp/rclcpp.hpp"
#include <termios.h>
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"

class ThrustSetpointsPublisher : public rclcpp::Node
{
public:
    ThrustSetpointsPublisher() : Node("pub_setpoints")
    {
        // Initialize subscribers
        sub_pilot_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/iris/controller/cmd_vel", 10,
            std::bind(&ThrustSetpointsPublisher::pilotCallback, this, std::placeholders::_1));

        // Initialize publishers
        pub_thrusters_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/iris/controller/thruster_setpoints", 10);

        // Create timer for control loop
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ThrustSetpointsPublisher::controlLoop, this));

        // Initialize variables
        yaw_cmd = 0.0;
        pitch_cmd = 0.0;
        descent_cmd = 0.0;
        fwd_cmd = 0.0;
        lat_cmd = 0.0;
        total_thrust = 0.0;
        setpoints_ = std::vector<double>(8, 0.0);
        thrust_ = std::vector<double>(8, 1.0);

        // Calculate moments of inertia
        Iz_ = (0.20)*MASS*(pow(0.5,2)+pow(0.3,2));
        Iy_ = MASS*(pow(0.70,2)+pow(0.50,2))/12.0;
        ldivIz_ = l_/Iz_;
        hdivIy_ = h_/Iy_;
        RCLCPP_INFO(this->get_logger(), "ldivIZ = %f", ldivIz_);
        RCLCPP_INFO(this->get_logger(), "IZ = %f", Iz_);
    }

private:
    void pilotCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        descent_cmd = msg->linear.z;
        fwd_cmd = msg->linear.x;
        lat_cmd = msg->linear.y;
        yaw_cmd = msg->angular.z;
    }

    void controlLoop()
    {
        total_thrust = sqrt(pow(fwd_cmd,2)+pow(lat_cmd,2));
        //saturate to total thrust available
        if (total_thrust >= 200.0) total_thrust = 200.0;
        if (total_thrust <= -200.0) total_thrust = -200.0;

        thrust_[0] = (total_thrust/4)+(yaw_cmd/(4*ldivIz_));
        thrust_[1] = (total_thrust/4)-(yaw_cmd/(4*ldivIz_));
        thrust_[2] = (total_thrust/4)-(yaw_cmd/(4*ldivIz_));
        thrust_[3] = (total_thrust/4)+(yaw_cmd/(4*ldivIz_));

        // Small saturation correction
        if ((total_thrust/4)+(yaw_cmd/(4*ldivIz_)) > 50.0)
        {
            thrust_[0] = 50.0;
            thrust_[1] = 50.0-(yaw_cmd/(2*ldivIz_));
            thrust_[2] = 50.0-(yaw_cmd/(2*ldivIz_));
            thrust_[3] = 50.0;
        }
        if ((total_thrust/4)-(yaw_cmd/(4*ldivIz_)) < -50.0)
        {
            thrust_[0] = -50.0+(yaw_cmd/(2*ldivIz_));
            thrust_[1] = -50.0;
            thrust_[2] = -50.0;
            thrust_[3] = -50.0+(yaw_cmd/(2*ldivIz_));
        }

        // Apply descent speed
        thrust_[4] = -descent_cmd-thrust_buoyancy_offset;
        thrust_[5] = -descent_cmd-thrust_buoyancy_offset;  
        thrust_[6] = 0;
        thrust_[7] = 0;

        if (std::abs(thrust_[4]) > 50.0)
        {
            for (size_t i = 4; i < 6; i++)
            {
                thrust_[i] = (50.0/(std::abs(thrust_[4])))*thrust_[i];
            }
        }

        RCLCPP_INFO(this->get_logger(), "thrust: %f,%f,%f,%f,%f,%f,%f,%f", 
            thrust_[0], thrust_[1], thrust_[2], thrust_[3],
            thrust_[4], thrust_[5], thrust_[6], thrust_[7]);

        // Normalize to setpoints
        for (size_t i = 0; i < 8; i++)
        {
            setpoints_[i] = thrust_[i]/50.0;
        }

        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data = setpoints_;
        pub_thrusters_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "setpoints: %f,%f,%f,%f,%f,%f,%f,%f",
            setpoints_[0], setpoints_[1], setpoints_[2], setpoints_[3],
            setpoints_[4], setpoints_[5], setpoints_[6], setpoints_[7]);
    }

    // Constants
    const double MASS = 20.0;
    const double l_ = 0.1653;
    const double h_ = 0.089;
    const double thrust_buoyancy_offset = 0.0;

    // Member variables
    double yaw_cmd, pitch_cmd, descent_cmd, fwd_cmd, lat_cmd;
    double total_thrust;
    double Iz_, Iy_, ldivIz_, hdivIy_;
    std::vector<double> setpoints_;
    std::vector<double> thrust_;

    // ROS interfaces
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

