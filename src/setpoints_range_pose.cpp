#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <termios.h>
#include <tf/tf.h>
double yaw_cmd, descent_speed, fwd_speed, lat_speed;
void pilotCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    descent_speed = msg->linear.z;
    fwd_speed = msg->linear.x;
    lat_speed = msg->linear.y;
    yaw_cmd = msg->angular.z;
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "pub_setpoints");

  ros::NodeHandle nh, nh_;
  
  //subcribe pilot
  ros::Subscriber sub_pilot;
  sub_range = nh.subscribe("/iris/controller/cmd_vel",1000, pilotCallback);
  //setpoints publisher
  ros::Publisher pub_thrusters;
  pub_thrusters = nh_.advertise<std_msgs::Float64MultiArray>("/iris/controller/thruster_setpoints", 1000);

  ros::Rate loop_rate(10);

  std::vector<double> setpoints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  
  while (ros::ok())
  {
      // setpoints[0] = fwd_speed+(lat_speed/4) - (yaw_cmd)/4;
      // setpoints[1] = fwd_speed-(lat_speed)/4 + (yaw_cmd)/4;
      // setpoints[2] = fwd_speed-(lat_speed)/4 + (yaw_cmd)/4;
      // setpoints[3] = fwd_speed+(lat_speed)/4 - (yaw_cmd)/4;
      setpoints[0] = fwd_speed - (yaw_cmd)/2;
      setpoints[1] = fwd_speed + (yaw_cmd)/2;
      setpoints[2] = fwd_speed + (yaw_cmd)/2;
      setpoints[3] = fwd_speed - (yaw_cmd)/2;
      setpoints[4] = -descent_speed;
      setpoints[5] = -descent_speed;
      // setpoints[6] = -lat_speed-yaw_cmd;
      // setpoints[7] = -lat_speed+yaw_cmd;
      setpoints[6] = - yaw_cmd/2;
      setpoints[7] = yaw_cmd/2;

      // publish setpoints
      std_msgs::Float64MultiArray msg_setpoints;
      msg_setpoints.data = setpoints;
      pub_thrusters.publish(msg_setpoints);
      ros::spinOnce();
      loop_rate.sleep();

  }
  
  return 0;
}

