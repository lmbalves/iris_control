
/**
 * @file range_calculator.cpp
 * @author Luis Alves (lmbalves@gmail.com)
 * @brief This node calculates the range and using a lyapunov approach calculates and
 *        publishes a commanded yaw
 * @version 0.1
 * @date 2021-10-04
 * 
 * 
 */

#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <stonefish_ros/DVL.h>
#include <tf/tf.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <math.h>

double speed, radius, des_yaw, des_z, roll, pitch, yaw, com_yaw, rel_heading, speed_X, speed_Y;
double k_z = 1.1;
double k = 4;
double k_yaw = 0.03;
double alpha  = 1;

void quaternionCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
  tf::Quaternion q;
  tf::quaternionMsgToTF(msg->orientation, q);
  tf::Matrix3x3(q).getRPY(roll,pitch,yaw);
  
}

void dvl_Callback(const stonefish_ros::DVL::ConstPtr &msg)
{
  speed_X = msg->velocity.x;
  speed_Y = msg->velocity.y;
  speed = sqrt( pow( speed_X, 2 ) + pow( speed_Y, 2 ));
}
void Callback(const visualization_msgs::MarkerArray::ConstPtr &msg)
{
  // size_t it;
  // size_t size = msg->markers.begin() - msg->markers.end();
  // for (it == 0; it != size; ++it)
  // {
    // speed = 1;
    radius = 5;
    visualization_msgs::Marker marker = msg->markers[0];
    ROS_INFO("X = %f", marker.pose.position.x);
    ROS_INFO("Y = %f", marker.pose.position.y);
    ROS_INFO("Z = %f", marker.pose.position.z);
    double range = sqrt( pow( marker.pose.position.x, 2 ) + pow( marker.pose.position.y, 2 ));
    ROS_INFO("Range = %f", range);
    // double X_dot_d = ( speed / ( (range + 0.01) * ( pow(range,2) + pow(radius, 2) ) ) ) * ( -(marker.pose.position.x) * ( pow(range,2) - pow(radius,2) ) - ( marker.pose.position.y * (2 * range * radius) ) );
    // double Y_dot_d = ( speed / ( (range + 0.01) * ( pow(range,2) + pow(radius, 2) ) ) ) * ( -(marker.pose.position.y) * ( pow(range,2) - pow(radius,2) ) + ( marker.pose.position.x * (2 * range * radius) ) );
    // des_yaw = atan2(Y_dot_d, X_dot_d);
    des_z = marker.pose.position.z;
    rel_heading = (M_PI_2-atan2(marker.pose.position.y, marker.pose.position.x));
    if(rel_heading>=M_PI){ rel_heading-=2*M_PI;}
    else if(rel_heading<=M_PI){rel_heading+=2*M_PI;}
    ROS_INFO("Relative heading = %f", rel_heading);
    if (range > (1.1 * radius))
    {
      com_yaw =  rel_heading - ((5 * M_PI) / 6) + (( speed / range ) * sin(yaw - rel_heading)); 
    }
    else
    {
      des_yaw = rel_heading - M_PI_2 - (M_PI / 3) * pow((range - radius ) / radius, k);
      com_yaw = des_yaw  - ( speed / (alpha * range) ) * sin(yaw - rel_heading) - ((k * speed*M_PI) / (3 * pow(radius,k) * alpha )) * pow(range, k -1) * cos(yaw - rel_heading);
      com_yaw = com_yaw + M_PI_2;
    }
    ROS_INFO("com_yaw = %f", com_yaw);
    ROS_INFO("roll: %f pitch: %f yaw: %f", roll, pitch, yaw);
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "range_calculator");

  ros::NodeHandle n, np, nh, nhs;
  ros::Subscriber sub_imu = nh.subscribe("/iris/navigator/imu", 1000, quaternionCallback);
  ros::Subscriber sub_dvl = nhs.subscribe("/iris/odometry/dvl", 1000, dvl_Callback);
  ros::Subscriber sub = n.subscribe("/iris/navigator/usbl", 1000, Callback);
  ros::Publisher pub = np.advertise<geometry_msgs::Twist>("/iris/controller/cmd_vel", 1);
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    geometry_msgs::Twist msg;
    msg.linear.z = -k_z * des_z;
    msg.angular.z = -k_yaw * com_yaw;
    pub.publish(msg);
    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
