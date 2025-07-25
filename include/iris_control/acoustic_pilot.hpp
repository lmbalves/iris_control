#ifndef IRIS_CONTROL_ACOUSTIC_PILOT_HPP
#define IRIS_CONTROL_ACOUSTIC_PILOT_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace iris_control
{

class AcousticPilot : public rclcpp::Node
{
public:
  AcousticPilot();

private:
  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr cmd_pose_pub_;
  
  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  
  // Callback methods
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  
  // TF broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

} // namespace iris_control

#endif // IRIS_CONTROL_ACOUSTIC_PILOT_HPP