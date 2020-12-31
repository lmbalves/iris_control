#include <ros/ros.h>
#include <cola2_msgs/Setpoints.h>
#include <sensor_msgs/Joy.h>
#include <termios.h>
#include <csignal>
#include <cstdio>
#include <cstring>


std::map<int, std::vector<double>> thrusters
{
  {32, {0,0,0,0,0,0,0,0}},  //space bar -> COMPLETE STOP
  //-> HEAVE
  {87, {1,1,1,1,0,0,0,0}}, //W          
  {119, {1,1,1,1,0,0,0,0}}, //w
  {88, {-1,-1,-1,-1,0,0,0,0}}, //X
  {120, {-1,-1,-1,-1,0,0,0,0}}, //x
  //-> SWAY 
  {68, {0,0,0,0,0,0,1,-1}}, //D          
  {100, {0,0,0,0,0,0,1,-1}}, //d
  {65, {0,0,0,0,0,0,-1,1}}, //A
  {97, {0,0,0,0,0,0,-1,1}}, //a
  //-> SURGE
  {65, {0,0,0,0,1,1,0,0}}, //key up     
  {66, {0,0,0,0,-1,-1,0,0}}, //key down
  //PITCH
  {73, {1,1,0,0,0,0,0,0}}, //I
  {105, {1,1,0,0,0,0,0,0}}, //i
  {44, {0,0,1,1,0,0,0,0}}, //,
  //YAW
  {74, {0,1,1,0,0,0,0,0}}, //J
  {106, {0,1,1,0,0,0,0,0}}, //j
  {76, {1,0,0,1,0,0,0,0}}, //L
  {108, {1,0,0,1,0,0,0,0}}, //l
};

std::map<int, std::vector<double>> thruster_speed{
  {50, {1.9}}, //2, SPEED UP
  {49, {0.9}}, //1, SPEED DOWN
};

double t0(0), t1(0), t2(0), t3(0), t4(0), t5(0), t6(0), t7(0);
double speed = 1;

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
  ros::init(argc, argv, "pub_setpoints");
  ros::NodeHandle nh_;
  std::size_t setpoints_sent_;
  ros::Publisher pub_thrusters_ = nh_.advertise<cola2_msgs::Setpoints>("/controller/thruster_setpoints", 1000);
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
        speed = speed * thruster_speed[key][0];
      }
      
      //update
      setpoints[0] = t0 * speed;
      setpoints[1] = t1 * speed;
      setpoints[2] = t2 * speed;
      setpoints[3] = t3 * speed;
      setpoints[4] = t4 * speed;
      setpoints[5] = t5 * speed;
      setpoints[6] = t6 * speed;
      setpoints[7] = t7 * speed;

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

