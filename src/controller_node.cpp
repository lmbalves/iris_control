#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio.hpp>

static const std::string OPENCV_WINDOW_ORIGINAL = "Original Image";
static const std::string OPENCV_WINDOW_FILTERED = "Filtered Image";

void image_Callback(const sensor_msgs::Image::ConstPtr& msg)
{
    //Pull image in RGB using cv_bridge from OpenCv
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s ", e.what());
        ROS_INFO("ERROR");
        return;
    }

    //Convert to HSV in order to filter color
    cv::Mat hsv_image;
    cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);
    cv::Mat lower_red_hue_range;
    cv::Mat upper_red_hue_range;
    cv::inRange(hsv_image, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), lower_red_hue_range);
    cv::inRange(hsv_image, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), upper_red_hue_range);

    //Combine the two images
    cv::Mat red_hue_image;
    cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);
    cv::GaussianBlur(red_hue_image, red_hue_image, cv::Size(9, 9), 2, 2);
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;

    findContours( red_hue_image, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    
    drawContours(red_hue_image, contours, -1, (0, 255, 255),4);

    cv::imshow(OPENCV_WINDOW_ORIGINAL, cv_ptr->image);
    cv::waitKey(3);
    cv::imshow(OPENCV_WINDOW_FILTERED, red_hue_image);
    cv::waitKey(3);

}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "imagecontrol");
    ros::NodeHandle nh;

	ros::Subscriber sub_img_quad=nh.subscribe("/iris/proscilica_front/image_color", 1, image_Callback);

    ros::Rate loop_rate(20);

    while (ros::ok())
    {

        ros::spinOnce();

        loop_rate.sleep();

    }
 return 0;
}