#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Bool.h>


class Captain{
public:
    Captain(ros::NodeHandle *nh)
    {
        m_subVisualDetection = nh->subscribe("/iris/captain/visual_detection",
                                          1000, &Captain::visualBoolCallback, this);
        m_pubNavigationMode = nh->advertise<std_msgs::Bool>("/iris/captain/navigation_mode", 1000);
    }

    void visualBoolCallback(const std_msgs::Bool::ConstPtr &boolMsg);

private:
    ros::Subscriber m_subVisualDetection;
    ros::Publisher m_pubNavigationMode;

    bool m_visualNavigation = false;

};