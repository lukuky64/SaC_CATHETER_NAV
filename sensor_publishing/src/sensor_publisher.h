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



class sensorPublish
{
public:
    // Constructor: Initialises the sensorPublish class
    sensorPublish(ros::NodeHandle& nh, std::string baseFile_);

    // Destructor: Cleans up resources used by the sensorPublish class
    ~sensorPublish();

    // Publishes an image message
    void imagePublish(const sensor_msgs::ImagePtr msg);

    // Publishes an EM PoseWithCovarianceStamped message
    void EMPublish(geometry_msgs::PoseWithCovarianceStamped msg);

    // Returns the total number of images/data points for iteration
    int getTotalDataCount();

    // Sets up file reading parameters, like directory paths
    void setupFileRead(std::string baseFile_);

    // Reads an image file based on the count and returns an ImagePtr message
    sensor_msgs::ImagePtr readImage(int count_);

    // Checks if a file exists at the given path and returns a boolean result
    bool fileExists(const std::string& filename);

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

    void setPaused_state(bool state);

private:
    ros::NodeHandle nh_;               // ROS NodeHandle for managing ROS-related operations
    ros::Publisher image_pub_;         // ROS Publisher for image messages
    ros::Publisher EM_pub_;            // ROS Publisher for EM PoseWithCovarianceStamped messages
    int totalDataCount_;               // Total count of data points (for images and EM data)
    std::string folderDirectory_;      // Directory path where data files are located
    ros::Time current_time;            // Current ROS time
    int refresh_rate;                  // Rate at which messages are published
    bool paused_;                      // Reflects the state of the publisher, paused or playing
    //ros::ServiceServer paused_service; // service that changes the paused_ state variable
};

#endif // IMAGE_PROCESSOR_H
