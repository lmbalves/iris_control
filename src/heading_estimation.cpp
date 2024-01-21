#include "ros/ros.h"
#include <sensor_msgs/Imu.h>
#include <stonefish_ros/DVL.h>
#include <stonefish_ros/BeaconInfo.h>
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <cmath>
#include <cstdlib>
#include <random>

double roll, pitch, yaw;
double altitude, dvl_speed;
double estimated_heading;
double measured_range;

struct Particle
{
    double heading;
    double weight;
};

geometry_msgs::Vector3 linear_acceleration;
geometry_msgs::Vector3 velocity;
geometry_msgs::Vector3 displacement;
geometry_msgs::Vector3 actual_heading;

std::vector<Particle> particles;

// Particle filter parameters
const int num_particles = 1000;
const double motion_model_std_dev = 0.1;
const double measurement_std_dev = 0.5;

tf2_ros::Buffer tf_buffer;

double time_interval = 0.1;

void initializeParticles()
{
    particles.resize(num_particles);

    double prior_mean = M_PI_4l;
    double prior_std_dev = 0.05;

    std::default_random_engine generator;
    std::normal_distribution<double> distribution(prior_mean, prior_std_dev);

    double sum_weights = 1.0;

    for (int i = 0; i < num_particles; ++i)
    {
        particles[i].heading = distribution(generator);

        particles[i].weight = exp(-0.5 * pow((particles[i].heading - prior_mean) / prior_std_dev, 2));

        sum_weights += particles[i].weight;
    }

    for (int i = 0; i < num_particles; ++i)
    {
        particles[i].weight /= sum_weights;
    }
}

void predictParticles()
{
    for (auto &particle : particles)
    {
        particle.heading += (static_cast<double>(rand()) / RAND_MAX - 0.5) * motion_model_std_dev;
    }
}

void updateParticleWeights()
{
    for (auto &particle : particles)
    {
        double measurement_likelihood = exp(-0.5 * pow((particle.heading - measured_range) / measurement_std_dev, 2));
        particle.weight *= measurement_likelihood;
    }
}

void resampleParticles()
{
    std::vector<Particle> resampled_particles;

    for (size_t i = 0; i < particles.size(); ++i)
    {
        double rand_value = static_cast<double>(rand()) / RAND_MAX;
        double cumulative_weight = 0.0;

        for (const auto &particle : particles)
        {
            cumulative_weight += particle.weight;
            if (rand_value <= cumulative_weight)
            {
                resampled_particles.push_back(particle);
                break;
            }
        }
    }

    particles = resampled_particles;
}

double estimateHeading()
{
    for (const auto &particle : particles)
    {
        estimated_heading += particle.heading * particle.weight;
    }

    return estimated_heading;
}

void quaternionCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->orientation, q);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    linear_acceleration = msg->linear_acceleration;

    velocity.x = 0.0;
    velocity.y = 0.0;
    velocity.z = 0.0;

    velocity.x += linear_acceleration.x * time_interval;
    velocity.y += linear_acceleration.y * time_interval;
    velocity.z += linear_acceleration.z * time_interval;

    displacement.x += velocity.x * time_interval;
    displacement.y += velocity.y * time_interval;
    displacement.z += velocity.z * time_interval;

    // double X_dot_d = ( speed / ( (range + 0.01) * ( pow(range,2) + pow(radius, 2) ) ) ) * ( -(marker.pose.position.x) * ( pow(range,2) - pow(radius,2) ) - ( marker.pose.position.y * (2 * range * radius) ) );
    // double Y_dot_d = ( speed / ( (range + 0.01) * ( pow(range,2) + pow(radius, 2) ) ) ) * ( -(marker.pose.position.y) * ( pow(range,2) - pow(radius,2) ) + ( marker.pose.position.x * (2 * range * radius) ) );
    // des_yaw = atan2(Y_dot_d, X_dot_d);
}


void dvlCallback(const stonefish_ros::DVL::ConstPtr &msg)
{
    altitude = msg->altitude + 1;
    ROS_INFO("altitude: %f", altitude);
    ROS_INFO("roll: %f pitch: %f yaw: %f", roll, pitch, yaw);
    dvl_speed = sqrt(pow(msg->velocity.x, 2) + pow(msg->velocity.y, 2));
}

void rangeCallback(const stonefish_ros::BeaconInfo::ConstPtr &msg)
{
    measured_range = msg->range;
    ROS_INFO("elevation: %f", msg->elevation);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "heading_estimator");
    ros::NodeHandle nh;
    ros::NodeHandle nhp;

    ros::Subscriber sub_dvl = nh.subscribe("/iris/navigator/dvl", 1000, dvlCallback);
    ros::Subscriber sub_imu = nh.subscribe("/iris/navigator/imu", 1000, quaternionCallback);
    ros::Subscriber sub_range = nh.subscribe("/iris/navigator/usbl/beacon_info", 1000, rangeCallback);

    ros::Publisher pub_range = nhp.advertise<geometry_msgs::Twist>("/iris/estimated_heading", 1);

    initializeParticles();

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        geometry_msgs::Twist heading_msg;

        predictParticles();
        updateParticleWeights();
        resampleParticles();

        estimated_heading = estimateHeading();

        ROS_INFO("Estimated heading = %f", estimated_heading);
        ROS_INFO("Range: %f", measured_range);

        heading_msg.angular.z = estimated_heading;

        pub_range.publish(heading_msg);
        ros::spinOnce();

        loop_rate.sleep();
    }
    return 0;
}