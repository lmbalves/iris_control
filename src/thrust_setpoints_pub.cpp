#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <termios.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>

double yaw_cmd, pitch_cmd, descent_speed, fwd_speed, lat_speed;
double total_thrust;
double MASS = 30.0;
double thrust_buoyancy_offset = 20.0;
double l = 0.1653;


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

  ros::NodeHandle nh;
  
  //subcribe pilot
  ros::Subscriber sub_pilot;
  sub_pilot = nh.subscribe("/iris/controller/cmd_vel",1000, pilotCallback);
  //setpoints publisher
  ros::Publisher pub_thrusters;
  pub_thrusters = nh.advertise<std_msgs::Float64MultiArray>("/iris/controller/thruster_setpoints", 1000);

  ros::Rate loop_rate(10);

  std::vector<double> setpoints(8, 0.0);
  std::vector<double> thrust(8, 1.0);
  double Iz = (0.20)*MASS*(pow(0.5,2)+pow(0.3,2)); //elipsoid
  double ldivIz = l/Iz;
  ROS_INFO("ldivIZ = %f", ldivIz);

  ROS_INFO("IZ = %f", Iz);
  while (ros::ok())
  {
    total_thrust = sqrt(pow(fwd_speed,2)+pow(lat_speed,2));
    if (total_thrust >= 200.0) total_thrust = 200.0;
    if (total_thrust <= -200.0) total_thrust = -200.0;
    thrust[0] = (total_thrust/4)+(yaw_cmd/(4*ldivIz));
    thrust[1] = (total_thrust/4)-(yaw_cmd/(4*ldivIz));
    thrust[2] = (total_thrust/4)-(yaw_cmd/(4*ldivIz));
    thrust[3] = (total_thrust/4)+(yaw_cmd/(4*ldivIz));
    if ((total_thrust/4)+(yaw_cmd/(4*ldivIz)) > 50.0)
    {
        thrust[0] = 50.0;
        thrust[1] = 50.0-(yaw_cmd/(2*ldivIz));
        thrust[2] = 50.0-(yaw_cmd/(2*ldivIz));
        thrust[3] = 50.0;
    }
    if ((total_thrust/4)-(yaw_cmd/(4*ldivIz)) < -50.0)
    {
        thrust[0] = -50.0+(yaw_cmd/(2*ldivIz));
        thrust[1] = -50.0;
        thrust[2] = -50.0;
        thrust[3] = -50.0+(yaw_cmd/(2*ldivIz));
    }
    thrust[4] = -descent_speed-thrust_buoyancy_offset;
    thrust[5] = -descent_speed-thrust_buoyancy_offset;
    thrust[6] = 0;
    thrust[7] = 0;
    ROS_INFO("thrust: %f,%f,%f,%f,%f,%f,%f,%f", thrust[0], thrust[1],thrust[2],thrust[3],thrust[4],thrust[5],thrust[6],thrust[7]);
    
    if (std::abs(thrust[4]) > 50.0 )
    {
        for (size_t i = 4; i < 6; i++)
        {
            thrust[i] = (50.0/(std::abs(thrust[4])))*thrust[i];
        }
    }
   
    for (size_t i = 0; i < 8; i++)
    {
        setpoints[i]=thrust[i]/50;
    }

    std_msgs::Float64MultiArray msg_setpoints;
    msg_setpoints.data = setpoints;
    pub_thrusters.publish(msg_setpoints);

    ROS_INFO("setpoints: %f,%f,%f,%f,%f,%f,%f,%f", setpoints[0], setpoints[1],setpoints[2],setpoints[3],setpoints[4],setpoints[5],setpoints[6],setpoints[7]);
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}

