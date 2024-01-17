#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include "setpoints_visual_navigation.hpp"


void VisualNavigation::captainCallback(const std_msgs::Bool::ConstPtr &decisionMsg){
    m_navigation = &decisionMsg->data;
    std::cout << "\n--------------";    
    std::cout << "\nVisual control" << m_navigation;    
    std::cout << "\n--------------\n";  
}

void VisualNavigation::imageDataCallback(const std_msgs::Float32MultiArray::ConstPtr& imageDataMsg){
    m_imageData = imageDataMsg->data;
    std::cout << "\n--------------";    
    std::cout << "\nCenter frame = (" << m_imageData[0] << "," << m_imageData[1] << ")";    
    std::cout << "\nCenter box = (" << m_imageData[2] << "," << m_imageData[3] << ")";    
    std::cout << "\n--------------\n";  
    thrusterControl(m_imageData);
}

/* imageData = [imageCenterX, imageCenterY, boxCenterX, boxCenterY,
                imageWidth, imageHeight, boxWidth, boxHeight] */
void VisualNavigation::thrusterControl(std::vector<float> imageData){
    std_msgs::Float64MultiArray msgSetpoints;
    /* SURGE CONTROL BASED ON BOX SIZE */
    float frameSize = imageData[4]+imageData[5];
    float boxSize = imageData[6]+imageData[7];

    float errorSize = frameSize - boxSize;
    float surgeIntegral = errorSize*DT;

    float surgeInput = KPY*errorSize + KIY*surgeIntegral;

    float surgeNormalized = surgeInput/frameSize;

    /* YAW AND HEAVE CONTROL BASED ON CENTER POINTS*/
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

    if (m_navigation == true){
        /* Surge input setpoints*/
        // m_setpoints[0] = surgeNormalized;
        // m_setpoints[1] = surgeNormalized;
        // m_setpoints[2] = surgeNormalized;
        // m_setpoints[3] = surgeNormalized;
        /* Heave input setpoints*/
        m_setpoints[4] = m_thrusterControlYN;
        m_setpoints[5] = m_thrusterControlYN;
        /* Yaw input setpoints */
        m_setpoints[6] = m_thrusterControlXN;
        m_setpoints[7] = m_thrusterControlXN;

        std::cout << "\n|||||||||||||||||";
        std::cout << "\nSurge = " << surgeNormalized;    
        std::cout << "\nHeave = " << m_thrusterControlYN;    
        std::cout << "\nYaw = " << m_thrusterControlXN;    
        std::cout << "\n|||||||||||||||||";

        msgSetpoints.data = m_setpoints;
        m_pubThrusters.publish(msgSetpoints);
    }
}


int main (int argc, char **argv){
    ros::init(argc, argv, "setpoints_visual_navigation");
    
    ros::NodeHandle nh;
    VisualNavigation irisVisualControl = VisualNavigation(&nh);
    ros::spin();
}