#ifndef ULTRASOUND_PUBLISHER_H
#define ULTRASOUND_PUBLISHER_H

// headers
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <fstream>
#include <unistd.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <limits>
#include "std_srvs/SetBool.h"
#include <boost/bind.hpp>
#include <thread>
#include <chrono>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>

class sensorPublish
{
public:
    // Constructor: Initialises the sensorPublish class
    sensorPublish(ros::NodeHandle &nh, std::string baseFile_);

    // Destructor: Cleans up resources used by the sensorPublish class
    ~sensorPublish();

    // Publishes an image message
    void imagePublish(const sensor_msgs::ImagePtr msg);

    // Publishes an EM PoseWithCovarianceStamped message
    void EMPublish(geometry_msgs::PoseWithCovarianceStamped msg);

    // Publishes a Point Cloud orientation PoseWithCovarianceStamped message
    void PCPPublish(geometry_msgs::PoseWithCovarianceStamped msg);

    // Returns the total number of images/data points for iteration
    int getTotalDataCount();

    // Sets up file reading parameters, like directory paths
    void setupFileRead(std::string baseFile_);

    // Reads an image file based on the count and returns an ImagePtr message
    sensor_msgs::ImagePtr readImage(int count_);

    // Checks if a file exists at the given path and returns a boolean result
    bool fileExists(const std::string &filename);

    // Reads an EM data file based on the count and returns a PoseWithCovarianceStamped message
    geometry_msgs::PoseWithCovarianceStamped readEM(int count_);

    // Sets covariance values to the msg variable
    void getCovariance(geometry_msgs::PoseWithCovarianceStamped &msg, int n);

    // Returns the refresh rate for publishing messages
    int getRefreshRate();

    // sets the state of the program (pause or play)
    bool setState(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

    // getter for paused_ variable
    bool getPaused_state();

    // setter for paused_ variable
    void setPaused_state(bool state);

    // calculates quaternion values for a line drawn between two points
    Eigen::Quaterniond calculateRotation(const Eigen::Vector3d &point1, const Eigen::Vector3d &point2);

    geometry_msgs::PoseWithCovarianceStamped createPCPose(geometry_msgs::PoseWithCovarianceStamped msg);

    double euclideanDistance(const Eigen::Vector3d& vec1, const Eigen::Vector3d& vec2);

private:
    ros::NodeHandle nh_;          // ROS NodeHandle for managing ROS-related operations
    ros::Publisher image_pub_;    // ROS Publisher for image messages
    ros::Publisher EM_pub_;       // ROS Publisher for EM PoseWithCovarianceStamped messages
    ros::Publisher PCP_Pub_;      // ROS Publisher for Point Cloud orientation PoseWithCovarianceStamped messages
    int totalDataCount_;          // Total count of data points (for images and EM data)
    std::string folderDirectory_; // Directory path where data files are located
    int refresh_rate;             // Rate at which messages are published
    bool paused_;                 // Reflects the state of the publisher, paused or playing
    // ros::ServiceServer paused_service; // service that changes the paused_ state variable

    Eigen::Vector3d current_position_;
    Eigen::Vector3d previous_position_;
    Eigen::Quaterniond current_quaternion_;
};

#endif // IMAGE_PROCESSOR_H
