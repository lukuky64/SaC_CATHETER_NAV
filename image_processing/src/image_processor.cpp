#include "image_processor.h"

// Constructor
ImageProcessor::ImageProcessor(ros::NodeHandle& nh) : nh_(nh)
{
    // Initialize the subscriber (!!!figure out what we want the queue size to be)
    image_sub_ = nh_.subscribe("image_topic", 10, &ImageProcessor::imageCallback, this);
}
        
// Destructor
ImageProcessor::~ImageProcessor()
{
    // Destructor code
}

// Callback function for image subscriber, currently converting bgr to grescale, change this to suit to suit ultrasonic data
void ImageProcessor::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        return;
    }

    // Perform image processing on cv_ptr->image using OpenCV
    // For example, convert to grayscale
    cv::Mat gray_image;
    cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_BGR2GRAY);

    // more processing...
}

// Main function
int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_processor");
    ros::NodeHandle nh;

    ImageProcessor ip(nh);

    ros::spin();
    return 0;
}
