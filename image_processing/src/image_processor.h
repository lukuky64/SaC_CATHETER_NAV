#ifndef IMAGE_PROCESSOR_H
#define IMAGE_PROCESSOR_H

// ROS headers
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

// OpenCV headers
#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>

// CvBridge header
#include <cv_bridge/cv_bridge.h>


// Struct for holding anisotropic diffusion parameters
struct AnisotropicDiffusionParams {
    float alpha;  // Time step for each iteration
    float K;      // Sensitivity to edges
    int niters;   // Number of iterations
};


class ImageProcessor
{
public:
    // Constructor
    ImageProcessor(ros::NodeHandle& nh);

    // Destructor
    ~ImageProcessor();

    // Callback function for image subscriber
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

    // remove colour from image
    cv::Mat isolateGray(cv::Mat input_image);

private:
    ros::NodeHandle nh_;
    ros::Subscriber image_sub_;
    ros::Publisher image_pub_;  

    AnisotropicDiffusionParams AD_params;
};

#endif // IMAGE_PROCESSOR_H
