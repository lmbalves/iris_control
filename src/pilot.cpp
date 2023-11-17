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
#include <tf/tf.h>
#include <math.h>
#include <stonefish_ros/DVL.h>
#include <geometry_msgs/Vector3.h>

// gains
double k_range = 0.2;
double k_z = 0.7;
double k_yaw = 0.15;


double desired_range = 2.0;
double desired_z = 2.0;

double time_interval = 0.1;

double range, altitude, prev_altitude;

double control_x, control_y, control_z;

double speed;

double des_yaw, roll, pitch, yaw, com_yaw, rel_heading;


geometry_msgs::Vector3 linear_acceleration;
geometry_msgs::Vector3 velocity;
geometry_msgs::Vector3 displacement;
geometry_msgs::Vector3 direction;
geometry_msgs::Vector3 desired_heading;

void Callback(const visualization_msgs::MarkerArray::ConstPtr &msg)
{
    visualization_msgs::Marker marker = msg->markers[0];

    // calculate the range to the beacon
    range = sqrt(pow(marker.pose.position.x, 2) + pow(marker.pose.position.y, 2) + pow(marker.pose.position.z, 2));
    double range_error = range - desired_range;
    ROS_INFO("altitude_xxx: %f", altitude);
    double z_error = desired_z - altitude;

    // lyapunov function based on the range error
    double lyapunov_func = 0.5 * k_range * pow(range_error, 2);
    double lyapunov_func_z = 0.5 * k_z* pow(z_error, 2);

    // derivative of the Lyapunov function
    double lyapunov_deriv = k_range * range_error;
    double lyapunov_deriv_z = k_z * z_error;
    double altitude_deriv = (altitude - prev_altitude) / time_interval;
    prev_altitude = altitude;

    desired_heading.x = 1.0 / lyapunov_func;
    desired_heading.y = 1.0 / lyapunov_func;
    rel_heading = (M_PI_2-atan2(desired_heading.y, desired_heading.x));

    if(rel_heading>=M_PI){ rel_heading-=2*M_PI;}
    else if(rel_heading<=M_PI){rel_heading+=2*M_PI;}
    if(rel_heading>=M_PI){ rel_heading-=2*M_PI;}
    else if(rel_heading<=M_PI){rel_heading+=2*M_PI;}
    ROS_INFO("Relative heading = %f", rel_heading);
    
    com_yaw =  rel_heading - ((5 * M_PI) / 6) + (( speed / range ) * sin(yaw - rel_heading)); 

    control_x = -lyapunov_deriv * range_error / range;
    control_y = -lyapunov_deriv * range_error / range;
    control_z = -lyapunov_deriv_z * z_error / desired_z;

    // monitoring
    ROS_INFO("lyapunov_func: %f", lyapunov_func);
    ROS_INFO("range_error: %f", range_error);
    ROS_INFO("lyapunov_func_z: %f", lyapunov_func_z);
    ROS_INFO("z_error: %f", z_error);
    ROS_INFO("com_yaw = %f", com_yaw);
    ROS_INFO("roll: %f pitch: %f yaw: %f", roll, pitch, yaw);
}

void quaternionCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
  tf::Quaternion q;
  tf::quaternionMsgToTF(msg->orientation, q);
  tf::Matrix3x3(q).getRPY(roll,pitch,yaw);
  linear_acceleration = msg->linear_acceleration;

  velocity.x += linear_acceleration.x * time_interval;
  velocity.y += linear_acceleration.y * time_interval;
  velocity.z += linear_acceleration.z * time_interval;

  displacement.x += velocity.x * time_interval;
  displacement.y += velocity.y * time_interval;
  displacement.z += velocity.z * time_interval;
}

void dvl_Callback(const stonefish_ros::DVL::ConstPtr &msg)
{
  altitude = msg->altitude;
  ROS_INFO("altitude: %f", altitude);
  speed = sqrt( pow( msg->velocity.x, 2 ) + pow( msg->velocity.y, 2 ));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pilot");
  ros::NodeHandle n, np, nh, nhq;

  // subscribe usbl, dvl and imu
  ros::Subscriber sub = n.subscribe("/iris/navigator/usbl", 1000, Callback);
  ros::Subscriber sub_dvl = nh.subscribe("/iris/navigator/dvl", 1000, dvl_Callback);
  ros::Subscriber sub_imu = nhq.subscribe("/iris/navigator/imu", 1000, quaternionCallback);
  ros::Publisher pub = np.advertise<geometry_msgs::Twist>("/iris/controller/cmd_vel", 1);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    // publish control commands
    geometry_msgs::Twist msg_control;
    msg_control.linear.x = -control_x;
    msg_control.linear.y = -control_y;
    msg_control.linear.z = -control_z;
    msg_control.angular.z = -k_yaw * com_yaw;
    pub.publish(msg_control);
    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}