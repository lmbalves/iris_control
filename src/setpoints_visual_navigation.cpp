#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include "setpoints_visual_navigation.hpp"

void VisualNavigation::imageCenterPointsCallback(const std_msgs::Float32MultiArray::ConstPtr& centerPointsMsg){
    m_imageCenterPointsData = centerPointsMsg->data;
    std::cout << "\n--------------";    
    std::cout << "\nCenter frame = (" << m_imageCenterPointsData[0] << "," << m_imageCenterPointsData[1] << ")";    
    std::cout << "\nCenter box = (" << m_imageCenterPointsData[2] << "," << m_imageCenterPointsData[3] << ")";    
    std::cout << "\n--------------\n";  
    thrusterControl(m_imageCenterPointsData);
}

void VisualNavigation::thrusterControl(std::vector<float> imageCenterPoints){
    /* Calculate error in X and Y*/
    m_xError = imageCenterPoints[0]-imageCenterPoints[2];
    m_yError = imageCenterPoints[1]-imageCenterPoints[3];

    /* thruster input values = constant*error */
    m_thrusterControlX = m_xConstant*m_xError;
    m_thrusterControlY = m_yConstant*m_yError;

    /* Normalize thruster input values*/
    m_thrusterControlXN = (m_thrusterControlX/imageCenterPoints[0]);
    m_thrusterControlYN = (m_thrusterControlY/imageCenterPoints[1]);

    std::cout << "\n|||||||||||||||||";
    std::cout << "\nThruster control = (" << m_thrusterControlXN << "," << m_thrusterControlYN << ")";    
    std::cout << "\n|||||||||||||||||";

    /* HEAVE */
    m_setpoints[4] = m_thrusterControlYN;
    m_setpoints[5] = m_thrusterControlYN;
    /* YAW */
    m_setpoints[6] = m_thrusterControlXN;
    m_setpoints[7] = m_thrusterControlXN;

    std_msgs::Float64MultiArray msgSetpoints;
    msgSetpoints.data = m_setpoints;
    m_pubThrusters.publish(msgSetpoints);
}


int main (int argc, char **argv){
    ros::init(argc, argv, "setpoints_visual_navigation");
    
    ros::NodeHandle nh;
    VisualNavigation imageCenterPoints = VisualNavigation(&nh);
    ros::spin();
}