#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Joy.h>
#include <termios.h>
#include <csignal>
#include <cstdio>
#include <cstring>


std::map<int, std::vector<double>> thrusters
{

  {67,{0,0,0,0,-0.15,-0.3,-0.5,0.3}},//t
  {116,{0,0,0,0,-0.15,-0.3,-0.5,0.3}},
  //{116,{0,0,0,0,-0.2,-0.5,-0.9,0.65}},

//{116,{0,0,0,0,-0.2,-0.5,-0.9,0.65}},

  //{70,{0,0,0,0,-0.2,-0.6,-1,0.8}},//f 
  //{102,{0,0,0,0,-0.2,-0.6,-1,0.8}}, 

  //{71,{0,0,0,0,-0.5,-0.2,0.9,-0.65}},//g frente baixo
  //{103,{0,0,0,0,-0.5,-0.2,0.9,-0.65}}, 

  {32, {0,0,0,0,0,0,0,0}},  //space bar -> COMPLETE STOP
  //-> HEAVE
  {87, {1,1,1,1,0,0,0,0}}, //W          		front
  {119, {1,1,1,1,0,0,0,0}}, //w				
  {88, {-1,-1,-1,-1,0,0,0,0}}, //X			back
  {120, {-1,-1,-1,-1,0,0,0,0}}, //x
  //-> SURGE
  {65, {0,0,0,0,1,1,0,0}}, //key up     		z control
  {66, {0,0,0,0,-1,-1,0,0}}, //key down			z control
  {68, {0,0,0,0,0,-0.4,0.25,0.25}},     //seta esquerda  yaw control 
  {67, {0,0,0,0,-0.4,0,-0.25,-0.25}},   //D  	seta direita	yaw control 
};

std::map<int, std::vector<double>> thruster_speed
{
  {49, {1.5}}, //1, SPEED UP
  {50, {0.66}}, //2, SPEED DOWN
  {51, {0}}, //3, SPEED = 1
};

double t0(0), t1(0), t2(0), t3(0), t4(0), t5(0), t6(0), t7(0);
double speed_iris = 0.5;

int getChar()
{
    int ch;
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    std::memcpy(&newt, &oldt, sizeof(struct termios));
    newt.c_lflag &= ~(ICANON | ECHO);
    newt.c_cc[VEOL] = 1;
    newt.c_cc[VEOF] = 2;
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pub_setpoints2");
  ros::NodeHandle nh_;
  std::size_t setpoints_sent_;
  ros::Publisher pub_thrusters_ = nh_.advertise<std_msgs::Float64MultiArray>("/iris/controller/thruster_setpoints", 1000);
  ros::Rate loop_rate(10);
  std::vector<double> setpoints = {0,0,0,0,0,0,0,0};

  int count = 0;
  
  while (ros::ok())
  {
      // Find which button is pressed
      int key = getChar();
     
      //key detection
      if (thrusters.count(key)==1){
        t0=thrusters[key][0];
        t1=thrusters[key][1];
        t2=thrusters[key][2];
        t3=thrusters[key][3];
        t4=thrusters[key][4];
        t5=thrusters[key][5];
        t6=thrusters[key][6];
        t7=thrusters[key][7];
      }
      //speed key detection
      if(thruster_speed.count(key)==1){
        speed_iris = speed_iris * thruster_speed[key][0];
        if (speed_iris==0){
            speed_iris=1;
        }
      }
      
      //update
      setpoints[0] = t0 * speed_iris;
      setpoints[1] = t1 * speed_iris;
      setpoints[2] = t2 * speed_iris;
      setpoints[3] = t3 * speed_iris;
      setpoints[4] = t4 * speed_iris;
      setpoints[5] = t5 * speed_iris;
      setpoints[6] = t6 * speed_iris;
      setpoints[7] = t7 * speed_iris;

      // Publish setpoints
      std_msgs::Float64MultiArray msg_setpoints;
      // msg_setpoints.header.seq = count;
      // msg_setpoints.header.stamp = ros::Time::now();  
      // msg_setpoints.header.frame_id = "/iris/base_link";
      msg_setpoints.data = setpoints;
      pub_thrusters_.publish(msg_setpoints);
      
      ros::spinOnce();
      loop_rate.sleep();
      ++count;
  }
  
  return 0;
}

