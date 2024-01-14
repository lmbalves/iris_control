#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

void centerPointsCallback(const std_msgs::Float32MultiArray::ConstPtr& centerPointsMsg){
    std::vector<float> data = centerPointsMsg->data;
    std::cout << "\n--------------";    
    std::cout << "\nCenter frame = (" << data[0] << "," << data[1] << ")";    
    std::cout << "\nCenter box = (" << data[2] << "," << data[3] << ")";    
    std::cout << "\n--------------\n";    
}

int main (int argc, char **argv){
    ros::init(argc, argv, "setpoints_visual_navigation");
    
    ros::NodeHandle nh;
    ros::Subscriber subCenterPoints = nh.subscribe("/iris/proscilica_front/image_center_points", 1000, centerPointsCallback);
    ros::spin();
}