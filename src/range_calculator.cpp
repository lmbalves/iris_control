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
#include <tf/tf.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <math.h>
double distance(double x1, double y1,
            double z1, double x2,
            double y2, double z2);
double speed, radius, des_yaw, des_z, roll, pitch, yaw;
double k_one = 0.5;
double k = 0.9;
// double distance(double x1, double y1,
//             double z1, double x2,
//             double y2, double z2)
// {
//     double d = sqrt(pow(x2 - x1, 2) +
//                 pow(y2 - y1, 2));

//     ROS_INFO(" Distance is %f", d);
//     return d;
// }

void quaternionCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
  tf::Quaternion q;
  tf::quaternionMsgToTF(msg->orientation, q);
  tf::Matrix3x3(q).getRPY(roll,pitch,yaw);
  
}
void Callback(const visualization_msgs::MarkerArray::ConstPtr &msg)
{
  // size_t it;
  // size_t size = msg->markers.begin() - msg->markers.end();
  // for (it == 0; it != size; ++it)
  // {
    speed = 1;
    radius = 10;
    visualization_msgs::Marker marker = msg->markers[0];
    ROS_INFO("X = %f", marker.pose.position.x);
    ROS_INFO("Y = %f", marker.pose.position.y);
    ROS_INFO("Z = %f", marker.pose.position.z);
    double range = sqrt( pow( marker.pose.position.x, 2 ) + pow( marker.pose.position.y, 2 ));
    ROS_INFO("Range = %f", range);
    double X_dot_d = ( speed / ( (range + 0.01) * ( pow(range,2) + pow(radius, 2) ) ) ) * ( -(marker.pose.position.x) * ( pow(range,2) - pow(radius,2) ) - ( marker.pose.position.y * (2 * range * radius) ) );
    double Y_dot_d = ( speed / ( (range + 0.01) * ( pow(range,2) + pow(radius, 2) ) ) ) * ( -(marker.pose.position.y) * ( pow(range,2) - pow(radius,2) ) + ( marker.pose.position.x * (2 * range * radius) ) );
    des_yaw = atan2(Y_dot_d, X_dot_d);
    des_z = marker.pose.position.z;
    // if(des_yaw>=M_PI){ des_yaw-=2*M_PI;}
    // else if(des_yaw<=M_PI){des_yaw+=2*M_PI;}
  // }
    ROS_INFO("des_yaw = %f", des_yaw);
    ROS_INFO("roll: %f pitch: %f yaw: %f", roll, pitch, yaw);
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "range_calculator");

  ros::NodeHandle n, np, nh;
  ros::Subscriber sub_imu = nh.subscribe("/iris/navigator/imu", 1000, quaternionCallback);
  ros::Subscriber sub = n.subscribe("/iris/navigator/usbl", 1000, Callback);
  ros::Publisher pub = np.advertise<geometry_msgs::Twist>("/iris/controller/cmd_vel", 1);
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    geometry_msgs::Twist msg;
    msg.linear.z = -k * des_z;
    msg.angular.z = -k_one * ((yaw - des_yaw));
    pub.publish(msg);
    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}