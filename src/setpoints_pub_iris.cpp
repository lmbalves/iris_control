#include <ros/ros.h>
#include <cola2_msgs/Setpoints.h>
#include <termios.h>

#include <csignal>
#include <cstdio>
#include <cstring>
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
  //const std::string ns_;
  ros::NodeHandle nh_;
  std::size_t setpoints_sent_;


  ros::Publisher pub_thrusters_;
  // Create publisher
  pub_thrusters_ = nh_.advertise<cola2_msgs::Setpoints>("iris/controller/thruster_setpoints", 1000);


 
  ros::Rate loop_rate(10);
  int count= 0;
  std::vector<double> setpoints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  double speed = 0.0;
  while (ros::ok())
  {
    // Forward, backward, turn left and turn right
    const int KEY_W = 87;
    const int KEY_w = 119;
    const int KEY_S = 83;
    const int KEY_s = 115;
    const int KEY_A = 65;
    const int KEY_a = 97;
    const int KEY_D = 68;
    const int KEY_d = 100;

    // For yaw and heave velocities - not used yet
    const int KEY_T = 84;
    const int KEY_t = 116;
    const int KEY_G = 71;
    const int KEY_g = 103;
    const int KEY_F = 70;
    const int KEY_f = 102;
    const int KEY_H = 72;
    const int KEY_h = 104;

    // Up and down 
    const int KEY_UP = 65;
    const int KEY_DOWN = 66;
    const int KEY_RIGHT = 67;
    const int KEY_LEFT = 68;

    // overall speed
    const int KEY_O = 79;
    const int KEY_o = 111;
    const int KEY_K = 75;
    const int KEY_k = 107;

    // Pitch mode - not used yet
    const int KEY_M = 77;
    const int KEY_m = 109;
    const int KEY_N = 78;
    const int KEY_n = 110;

    // Esc and Space control actions
    const int KEY_ESC = 27;
    const int KEY_OBRACKET = 91;
    const int KEY_SPACE = 32;
    const int KEY_DOT = 46;
    const int KEY_COMA = 44;

      // Find which button is pressed
      int key = getChar();


      // define setpoints
      if (key == KEY_SPACE)
      {
        setpoints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      }
      else if (key == KEY_W || key == KEY_w)
      {
        // setpoints = {0.25, 0.25, 0.25, 0.25, 0.0, 0.0, 0.0, 0.0};
        for (int i = 0; i <= 3 ; i++)
        {
          setpoints[i] = speed;
        }  
      }
      else if (key == KEY_S || key == KEY_s)
      {
        // setpoints = {-0.25, -0.25, -0.25, -0.25, 0.0, 0.0, 0.0, 0.0};
        for (int i = 0; i <= 3; i++)
        {
          setpoints[i] = -speed;
        }  
      }
      else if (key == KEY_A || key == KEY_a)
      {
        // setpoints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.25};
        setpoints[6] = -speed;
        setpoints[7] = -speed;
      }
      else if (key == KEY_D || key == KEY_d)
      {
        // setpoints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.25, -0.25};
        setpoints[6] = speed;
        setpoints[7] = speed;
      }
       else if (key == KEY_ESC)
      {
        if (getChar() == KEY_OBRACKET)
        {
          int arrow_key = getChar();
          if (arrow_key == KEY_UP)
          {
            // setpoints = {0.0, 0.0, 0.0, 0.0, 0.25, 0.25, 0.0, 0.0 };
            setpoints[4] = speed;
            setpoints[5] = speed; 
          }
          else if (arrow_key == KEY_DOWN)
          {
            // setpoints = {0.0, 0.0, 0.0, 0.0, -0.25, -0.25, 0.0, 0.0};
            setpoints[4] = -speed;
            setpoints[5] = -speed; 
          }
          else if (arrow_key == KEY_RIGHT)
          {
            // setpoints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, -0.25};
            setpoints[6] = speed;
            setpoints[7] = -speed; 
          }
          else if (arrow_key == KEY_LEFT)
          {
            // setpoints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.25, 0.25};
            setpoints[6] = -speed;
            setpoints[7] = speed;
          }
        }
      }
      else if (key == KEY_O || key == KEY_o)
      {
        speed = speed + 0.1;
      }
       else if (key == KEY_K || key == KEY_k)
      {
        speed = speed - 0.1;
      }
/*      else if (key == KEY_DOT)
      {
        buttons[11] = 1;
      }
      else if (key == KEY_COMA)
      {
        buttons[12] = 1;
      }
      else if (key == KEY_T || key == KEY_t)
      {
        buttons[13] = 1;
      }
      else if (key == KEY_G || key == KEY_g)
      {
        buttons[14] = 1;
      }
      else if (key == KEY_F || key == KEY_f)
      {
        buttons[15] = 1;
      }
      else if (key == KEY_H || key == KEY_h)
      {
        buttons[16] = 1;
      }
      else if (key == KEY_M || key == KEY_m)
      {
        buttons[17] = 1;
      }
      else if (key == KEY_N || key == KEY_n)
      {
        buttons[18] = 1;
      } */
      else
      {
        ROS_INFO_STREAM("Key code is " << key << ". Ignoring key");
        continue;
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

