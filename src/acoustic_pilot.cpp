/**
 * @file acoustic_pilot.cpp
 * @author Luis Alves (lmbalves@gmail.com)
 * @brief This node calculates the range and using a lyapunov approach calculates and
 *        publishes a commanded yaw
 * @version 0.2
 * @date 2021-10-04
 * 
 * 
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "stonefish_ros2/msg/dvl.hpp"  // Update with correct package name

#include <Eigen/Core>
#include <Eigen/Dense>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <math.h>

class AcousticPilot : public rclcpp::Node
{
public:
    AcousticPilot() : Node("acoustic_pilot")
    {
        // Initialize publishers
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/iris/controller/cmd_vel", 10);

        // Initialize subscribers
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/iris/navigator/imu", 10,
            std::bind(&AcousticPilot::quaternionCallback, this, std::placeholders::_1));

        dvl_sub_ = this->create_subscription<stonefish_ros2::msg::DVL>(
            "/iris/odometry/dvl", 10,
            std::bind(&AcousticPilot::dvlCallback, this, std::placeholders::_1));

        usbl_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "/iris/navigator/usbl", 10,
            std::bind(&AcousticPilot::markerCallback, this, std::placeholders::_1));

        // Create timer for control loop
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&AcousticPilot::controlLoop, this));

        // Initialize parameters
        k_z = 50.0;
        k_r = 50.0;
        k = 3.0;
        k_yaw = 0.33;
        alpha = 2.0;
    }

private:
    void quaternionCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    }

    void dvlCallback(const stonefish_ros2::msg::DVL::SharedPtr msg)
    {
        speed_X = msg->velocity.x;
        speed_Y = msg->velocity.y;
        speed = sqrt(pow(speed_X, 2) + pow(speed_Y, 2));
    }

    void markerCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
    {
        radius = 3.0;
        auto marker = msg->markers[0];
        RCLCPP_INFO(this->get_logger(), "X = %f", marker.pose.position.x);
        RCLCPP_INFO(this->get_logger(), "Y = %f", marker.pose.position.y);
        RCLCPP_INFO(this->get_logger(), "Z = %f", marker.pose.position.z);
        
        range_x = marker.pose.position.x;
        range_y = marker.pose.position.y;
        range = sqrt(pow(marker.pose.position.x, 2) + pow(marker.pose.position.y, 2));
        error_z = marker.pose.position.z;

        rel_heading = (M_PI_2 - atan2(marker.pose.position.y, marker.pose.position.x));
        if (rel_heading >= M_PI) {
            rel_heading -= 2 * M_PI;
        } else if (rel_heading <= M_PI) {
            rel_heading += 2 * M_PI;
        }

        if (range > (1.01 * radius)) {
            com_yaw = rel_heading - ((5 * M_PI) / 6) + ((speed / range) * sin(yaw - rel_heading));
        } else {
            des_yaw = rel_heading - M_PI_2 - (M_PI / 3) * pow((range - radius) / radius, k);
            com_yaw = des_yaw - (speed / (alpha * range)) * sin(yaw - rel_heading) -
                     ((k * speed * M_PI) / (3 * pow(radius, k) * alpha)) * pow(range, k - 1) * cos(yaw - rel_heading);
        }
    }

    void controlLoop()
    {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.z = k_z * error_z;
        msg.linear.x = k_r * range_x;
        msg.linear.y = k_r * range_y;
        msg.angular.z = k_yaw * com_yaw;
        cmd_vel_pub_->publish(msg);
    }

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<stonefish_ros2::msg::DVL>::SharedPtr dvl_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr usbl_sub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Variables
    double range, range_x, range_y, altitude_error, speed, radius;
    double des_yaw, error_z, roll, pitch, yaw, com_yaw, rel_heading;
    double speed_X, speed_Y;
    double k_z, k_r, k, k_yaw, alpha;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AcousticPilot>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
