#ifndef IMAGE_PROCESSOR_H
#define IMAGE_PROCESSOR_H

// ROS headers
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

// OpenCV headers
#include <opencv2/opencv.hpp>

// CvBridge header
#include <cv_bridge/cv_bridge.h>

class ImageProcessor
{
public:
    // Constructor
    ImageProcessor(ros::NodeHandle& nh);

    // Destructor
    ~ImageProcessor();

    // Callback function for image subscriber
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

private:
    ros::NodeHandle nh_;
    ros::Subscriber image_sub_;
};

#endif // IMAGE_PROCESSOR_H
