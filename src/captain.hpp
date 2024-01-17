#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Bool.h>


class Captain{
public:
    Captain(ros::NodeHandle *nh)
    {
        m_subVisualDetection = nh->subscribe("/iris/proscilica_front/ghost_detection_state",
                                          1000, &Captain::visualBoolCallback, this);
        m_pubNavigationMode = nh->advertise<std_msgs::Bool>("/iris/controller/navigation_mode", 1000);
    }

    void visualBoolCallback(const std_msgs::Bool::ConstPtr &boolMsg);

private:
    ros::Subscriber m_subVisualDetection;
    ros::Publisher m_pubNavigationMode;

    bool m_visualNavigation = false;

};