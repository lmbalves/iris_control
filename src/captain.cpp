#include <iostream>
#include <ros/ros.h>
// #include <std_msgs/Bool.h>
#include "captain.hpp"


void Captain::visualBoolCallback(const std_msgs::Bool::ConstPtr& boolMsg){
    m_visualNavigation = boolMsg->data;
    std::cout << "\n--------------";
    std::cout << "\nVisual Navigation -> " << m_visualNavigation;
    std::cout << "\n--------------\n";
    std_msgs::Bool visual;
    visual.data = m_visualNavigation;
    m_pubNavigationMode.publish(visual);
}

int main (int argc, char **argv){
    ros::init(argc, argv, "captain");
    
    ros::NodeHandle nh;
    Captain decideNavigation = Captain(&nh);
    ros::spin();
}