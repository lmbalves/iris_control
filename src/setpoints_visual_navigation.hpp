#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>

class VisualNavigation
{
public:
    VisualNavigation(ros::NodeHandle *nh)
    {
        m_subCenterPoints = nh->subscribe("/iris/proscilica_front/image_center_points",
                                          1000, &VisualNavigation::imageCenterPointsCallback, this);
        m_pubThrusters = nh->advertise<std_msgs::Float64MultiArray>("/iris/controller/thruster_setpoints", 1000);
    }
    /**
     * @brief Callback method that receives a vector containing the center point of the camera image 
     * and the center point of the highest scoring bounding box.
     */
    void imageCenterPointsCallback(const std_msgs::Float32MultiArray::ConstPtr &centerPointsMsg);
    
    /**
     * @brief Method that generates normalized inputs to the thrusters.
     */
    void thrusterControl(std::vector<float> imageCenterPoints);

private:
    ros::Subscriber m_subCenterPoints;
    ros::Publisher m_pubThrusters;
    std::vector<float> m_imageCenterPointsData;
    std::vector<double> m_setpoints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    float m_xError, m_yError;
    float m_thrusterControlX, m_thrusterControlY;
    float m_thrusterControlXN, m_thrusterControlYN;

    float Kpx = 0.1;
    float Kpy = 0.5;
    float Kix = 0.01;
    float Kiy = 0.001;
    float Kdx = 0.01;
    float Kdy = 0.01;

};