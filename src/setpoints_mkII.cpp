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
  //-> SURGE
  {87, {0.1,0.1,0.1,0.1,0,0,0,0}}, //W          
  {119, {0.1,0.1,0.1,0.1,0,0,0,0}}, //w
  {88, {-0.1,-0.1,-0.1,-0.1,0,0,0,0}}, //X
  {120, {-0.1,-0.1,-0.1,-0.1,0,0,0,0}}, //x
  //-> SWAY 
  {68, {0,0,0,0,0,0,-0.1,0.1}}, //D          
  {100, {0,0,0,0,0,0,-0.1,0.1}}, //d
  {65, {0,0,0,0,0,0,0.1,-0.1}}, //A
  {97, {0,0,0,0,0,0,0.1,-0.1}}, //a
  //-> HEAVE
  {73, {0,0,0,0,0.1,0.1,0,0}}, //I     
  {105, {0,0,0,0,0.1,0.1,0,0}}, //i
  {44, {0,0,0,0,-0.1,-0.1,0,0}}, //,
  //-> PITCH
  {82, {0.1,0.1,-0.1,-0.1,0,0,0,0}}, //R
  {114, {0.1,0.1,-0.1,-0.1,0,0,0,0}}, //r
  {86, {-0.1,-0.1,0.1,0.1,0,0,0,0}}, //V
  {118, {-0.1,-0.1,0.1,0.1,0,0,0,0}}, //v
  //->YAW ON POINT
  {74, {0,0,0,0,0,0,-0.1,-0.1}}, //J
  {106, {0,0,0,0,0,0,-0.1,-0.1}}, //j
  {76, {0,0,0,0,0,0,0.1,0.1}}, //L
  {108, {0,0,0,0,0,0,0.1,0.1}}, //l
  //->YAW WITH FOWARD/BACKWARD MOTION
  {81, {0.1,0,0,0.1,0,0,0,0}}, //Q
  {113, {0.1,0,0,0.1,0,0,0,0}}, //q
  {69, {0,0.1,0.1,0,0,0,0,0}}, //E
  {101, {0,0.1,0.1,0,0,0,0,0}}, //e
  {90, {-0.1,-0,-0,-0.1,0,0,0,0}}, //Z
  {122, {-0.1,-0,-0,-0.1,0,0,0,0}}, //z
  {67, {-0,-0.1,-0.1,-0,0,0,0,0}}, //C
  {99, {-0,-0.1,-0.1,-0,0,0,0,0}}, //c

};

double t0(0), t1(0), t2(0), t3(0), t4(0), t5(0), t6(0), t7(0);

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
  ros::Publisher pub_thrusters_ = nh_.advertise<cola2_msgs::Setpoints>("/iris/controller/thruster_setpoints", 1000);
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

      if(key==32){ //STOP ALL. Reset speed.
        setpoints[0] = t0;
        setpoints[1] = t1;
        setpoints[2] = t2;
        setpoints[3] = t3;
        setpoints[4] = t4;
        setpoints[5] = t5;
        setpoints[6] = t6;
        setpoints[7] = t7;
      }
      else{
        //Update. Adds to previous value.
        setpoints[0] = setpoints[0] + t0;
        setpoints[1] = setpoints[1] + t1;
        setpoints[2] = setpoints[2] + t2;
        setpoints[3] = setpoints[3] + t3;
        setpoints[4] = setpoints[4] + t4;
        setpoints[5] = setpoints[5] + t5;
        setpoints[6] = setpoints[6] + t6;
        setpoints[7] = setpoints[7] + t7; 
      }
      
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

