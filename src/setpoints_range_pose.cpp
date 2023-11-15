#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <termios.h>
#include <tf/tf.h>
double yaw_cmd, descent_speed, fwd_speed, lat_speed;
void rangeCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    descent_speed = msg->linear.z;
    fwd_speed = msg->linear.x;
    lat_speed = msg->linear.y;
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "pub_setpoints");

  ros::NodeHandle nh, nh_;


  ros::Subscriber sub_range;
  ros::Publisher pub_thrusters;
  // Create publisher
  pub_thrusters = nh_.advertise<std_msgs::Float64MultiArray>("/iris/controller/thruster_setpoints", 1000);
  sub_range = nh.subscribe("/iris/controller/cmd_vel",1000, rangeCallback);

 
  ros::Rate loop_rate(10);

  std::vector<double> setpoints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  
  while (ros::ok())
  {
      setpoints[0] = fwd_speed+lat_speed/4;
      setpoints[1] = fwd_speed-lat_speed/4;
      setpoints[2] = fwd_speed-lat_speed/4;
      setpoints[3] = fwd_speed+lat_speed/4;;
      setpoints[4] = -descent_speed;
      setpoints[5] = -descent_speed;
      setpoints[6] = -lat_speed;
      setpoints[7] = -lat_speed;

      // Publish setpoints
      std_msgs::Float64MultiArray msg_setpoints;
      msg_setpoints.data = setpoints;
      pub_thrusters.publish(msg_setpoints);
      ros::spinOnce();
      loop_rate.sleep();

  }
  
  return 0;
}

