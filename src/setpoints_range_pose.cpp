#include <ros/ros.h>
#include <cola2_msgs/Setpoints.h>
#include <termios.h>
#include <tf/tf.h>
double yaw_cmd, descent_speed;
void rangeCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    yaw_cmd = msg->angular.z;
    descent_speed = msg->linear.z;
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "pub_setpoints");
  //const std::string ns_;
  ros::NodeHandle nh, nh_;
  std::size_t setpoints_sent_;

  ros::Subscriber sub_range;
  ros::Publisher pub_thrusters_;
  // Create publisher
  pub_thrusters_ = nh_.advertise<cola2_msgs::Setpoints>("/iris/controller/thruster_setpoints", 1000);
  sub_range = nh.subscribe("/iris/controller/cmd_vel",1000, rangeCallback);

 
  ros::Rate loop_rate(10);
  int count= 0;
  std::vector<double> setpoints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  double speed = 0.5;
  
  while (ros::ok())
  {
      for (int i = 0; i <= 3; i++)
      {
          setpoints[i] = speed;
      }
      setpoints[0] = speed - (yaw_cmd)/4;
      setpoints[1] = speed + (yaw_cmd)/4;
      setpoints[2] = speed + (yaw_cmd)/4;
      setpoints[3] = speed - (yaw_cmd)/4;
      setpoints[4] = descent_speed;
      setpoints[5] = descent_speed;
      setpoints[6] = -yaw_cmd;
      setpoints[7] = yaw_cmd;
      // Publish setpoints
      cola2_msgs::Setpoints msg_setpoints;
      msg_setpoints.header.seq = count;
      msg_setpoints.header.stamp = ros::Time::now();  
      msg_setpoints.header.frame_id = "/iris/base_link";
      msg_setpoints.setpoints = setpoints;
      pub_thrusters_.publish(msg_setpoints);
      ros::spinOnce();
      loop_rate.sleep();
      ++count;
  }
  
  return 0;
}

