#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include "setpoints_visual_navigation.hpp"


void VisualNavigation::imageDataCallback(const std_msgs::Float32MultiArray::ConstPtr& imageDataMsg){
    m_imageData = imageDataMsg->data;
    std::cout << "\n--------------";    
    std::cout << "\nCenter frame = (" << m_imageData[0] << "," << m_imageData[1] << ")";    
    std::cout << "\nCenter box = (" << m_imageData[2] << "," << m_imageData[3] << ")";    
    std::cout << "\n--------------\n";  
    thrusterControl(m_imageData);
}


void VisualNavigation::thrusterControl(std::vector<float> imageData){
    /* Error in each image axis */
    m_xError = imageData[0]-imageData[2];
    m_yError = imageData[1]-imageData[3];

    /* Determine the integral value*/
    float xIntegral = m_xError*DT;
    float yIntegral = m_yError*DT;

    /* thruster input values = constant*error */
    m_thrusterControlX = KPX*m_xError + KIX*xIntegral;
    m_thrusterControlY = KPY*m_yError + KIY*yIntegral;

    /* Normalize thruster input values*/
    m_thrusterControlXN = -(m_thrusterControlX/imageData[0]);
    m_thrusterControlYN = (m_thrusterControlY/imageData[1]);

    std::cout << "\n|||||||||||||||||";
    std::cout << "\nThruster control = (" << m_thrusterControlXN << "," << m_thrusterControlYN << ")";    
    std::cout << "\n|||||||||||||||||";

    /* Heave input setpoints*/
    m_setpoints[4] = m_thrusterControlYN;
    m_setpoints[5] = m_thrusterControlYN;
    /* Yaw input setpoints */
    m_setpoints[6] = m_thrusterControlXN;
    m_setpoints[7] = m_thrusterControlXN;

    std_msgs::Float64MultiArray msgSetpoints;
    msgSetpoints.data = m_setpoints;
    m_pubThrusters.publish(msgSetpoints);
}


int main (int argc, char **argv){
    ros::init(argc, argv, "setpoints_visual_navigation");
    
    ros::NodeHandle nh;
    VisualNavigation irisVisualControl = VisualNavigation(&nh);
    ros::spin();
}