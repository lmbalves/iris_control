/**
 * @file pilot.cpp
 * @author Luis Alves (lmbalves@gmail.com)
 * @brief This node calculates the range and using a lyapunov approach calculates and
 *        publishes a commanded yaw
 * @version 0.1
 * @date 2021-10-04
 *
 *
 */

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/Imu.h>
#include <stonefish_ros/BeaconInfo.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf/tf.h>
#include <math.h>
#include <stonefish_ros/DVL.h>
#include <geometry_msgs/Vector3.h>
#include <cstdlib>
#include <random>

// gains
double k_range = 4;
double k_z = 0.9;
// double k_yaw = 0.15;
double alpha = M_PI;
double k = 2;

double desired_range = 5.0;
double desired_z = 5.0;

double time_interval = 0.1;

double altitude, prev_altitude;

double control_x, control_y, control_z;

double speed;

double des_yaw, com_yaw, rel_heading;
double roll, pitch, yaw;
double estimated_heading;
double measured_range;

void quaternionCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
  tf::Quaternion q;
  tf::quaternionMsgToTF(msg->orientation, q);
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
}

void Callback(const stonefish_ros::BeaconInfo::ConstPtr &msg)
{
  measured_range = msg->range;
  double range_error = measured_range - desired_range;
  double z_error = desired_z - altitude;

  // lyapunov function based on the range error
  double lyapunov_func = 0.5 * k_range * pow(range_error, 2);
  // double lyapunov_func_z = 0.5 * k_z* pow(z_error, 2);

  // derivative of the Lyapunov function
  double lyapunov_deriv = k_range * range_error;
  double lyapunov_deriv_z = k_z * z_error;
  // double altitude_deriv = (altitude - prev_altitude) / time_interval;
  // prev_altitude = altitude;

  // actual_heading.x = displacement.x;
  // actual_heading.y = displacement.y;
  // rel_heading = (M_PI_2-atan2(actual_heading.y, actual_heading.x));
  rel_heading = (M_PI-estimated_heading);
  if(rel_heading>=M_PI){ rel_heading-=M_2_PI;}
  else if(rel_heading<=M_PI){rel_heading+=M_2_PI;}
  // ROS_INFO("Relative heading = %f", rel_heading);

  if (measured_range > (1.1 * desired_range))
  {
    com_yaw = rel_heading - ((5 * M_PI) / 6) + ((speed / measured_range) * sin(yaw - rel_heading));
  }
  else
  {
    des_yaw = rel_heading - M_PI_2 - (M_PI / 3) * pow((measured_range - desired_range) / desired_range, k);
    com_yaw = des_yaw - (speed / (alpha * measured_range)) * sin(yaw - rel_heading) - ((k * speed * M_PI) / (3 * pow(desired_range, k) * alpha)) * pow(measured_range, k - 1) * cos(yaw - rel_heading);
    com_yaw = com_yaw + M_PI_2;
  }

  control_x = -lyapunov_deriv * range_error / measured_range;
  control_y = -lyapunov_deriv * range_error / measured_range;
  control_z = lyapunov_deriv_z;
  // monitoring
  // ROS_INFO("lyapunov_func: %f", lyapunov_func);
  // ROS_INFO("range_error: %f", range_error);
  // ROS_INFO("lyapunov_func_z: %f", lyapunov_func_z);
  // ROS_INFO("z_error: %f", z_error);
  ROS_INFO("range = %f", measured_range);
  ROS_INFO("range_error = %f", range_error);
  ROS_INFO("lyapunov_deriv = %f", lyapunov_deriv);
  ROS_INFO("com_yaw = %f", com_yaw);
  ROS_INFO("control_x = %f", control_x);
}

void dvl_Callback(const stonefish_ros::DVL::ConstPtr &msg)
{
  altitude = msg->altitude;
  // ROS_INFO("altitude: %f", altitude);
  speed = sqrt(pow(msg->velocity.x, 2) + pow(msg->velocity.y, 2));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pilot");

  ros::NodeHandle n0, n1, n2, n3;

  ros::Subscriber sub = n0.subscribe("/iris/navigator/usbl/beacon_info", 1000, Callback);
  ros::Subscriber sub_dvl = n1.subscribe("/iris/navigator/dvl", 1000, dvl_Callback);
  ros::Subscriber sub_imu = n2.subscribe("/iris/navigator/imu", 1000, quaternionCallback);

  ros::Publisher pub = n3.advertise<geometry_msgs::Twist>("/iris/controller/cmd_vel", 1);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    geometry_msgs::Twist msg_control;
    msg_control.linear.x = control_x;
    msg_control.linear.y = control_y;
    msg_control.linear.z = control_z;
    msg_control.angular.z = com_yaw;
    // msg_control.angular.z = -k_yaw * com_yaw;
    pub.publish(msg_control);
    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}