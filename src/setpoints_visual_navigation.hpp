#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <mutex>

#ifndef SETPOINTS_VISUAL_NAVIGATION
#define SETPOINTS_VISUAL_NAVIGATION

const float KPX = 0.2;
const float KPY = 0.6;
const float KIX = 0.01;
const float KIY = 0.01;
const float KDX = 0.01;
const float KDY = 0.01;
const float DT = 0.1;

const float KP_SURGE = 0.21;
const float KI_SURGE = 0.01;
const float KD_SURGE = 0.01;


class VisualNavigation{
public:
    VisualNavigation(ros::NodeHandle *nhVisual)
    {
        m_subCaptainDecision = nhVisual->subscribe("/iris/captain/navigation_mode",
                                          1000, &VisualNavigation::captainCallback, this);
        m_subDetectionData = nhVisual->subscribe("/iris/proscilica_front/ghost_detection_data",
                                          1000, &VisualNavigation::imageDataCallback, this);
        m_pubThrusters = nhVisual->advertise<std_msgs::Float64MultiArray>("/iris/controller/thruster_setpoints", 1000);
  
    }

    /**
     * @brief Callback method that subscribes a boolean value from the Captain node.
     * True -> Acoustic navigation interrupted, start visual navigation.
     * False -> Stop visual, resume acoustic navigation.
     */
    void captainCallback(const std_msgs::Bool::ConstPtr &decisionMsg);

    /**
     * @brief Callback method that subscribes to a topic published in IRIS Vision - setpoints_visual_navigation.
     * Message contains a vector with the center point of the image, the center point of the highest
     * scoring bounding box, image width, image height, bounding box width and height.
     */
    void imageDataCallback(const std_msgs::Float32MultiArray::ConstPtr &imageDataMsg);
    
    /**
     * @brief Method that generates normalized inputs to the thrusters based on received center points.
     * @param[in] imageData [image center point X, image center point Y, bounding box center point X,
     * bounding box center point Y, image width, image height, box width, box size].
     * X, Y are relative to the image, not the world.
     */
    void thrusterControl(std::vector<float> imageData);

private:
    ros::Subscriber m_subCaptainDecision;
    ros::Subscriber m_subDetectionData;
    ros::Publisher m_pubThrusters;
    std::vector<float> m_imageData;
    std::vector<double> m_setpoints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    bool m_navigation;
    float m_xError, m_yError;
    float m_lastErrorSurge;
    float m_thrusterControlX, m_thrusterControlY;
    float m_thrusterControlXN, m_thrusterControlYN;
};

#endif
