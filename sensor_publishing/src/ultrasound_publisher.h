#ifndef ULTRASOUND_PUBLISHER_H
#define ULTRASOUND_PUBLISHER_H

// headers
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <fstream>
#include <unistd.h>
#include <iostream>

class ultrasoundPublish
{
public:
    // Constructor
    ultrasoundPublish(ros::NodeHandle& nh);

    // Destructor
    ~ultrasoundPublish();

    // function for image publisher
    void imagePublish(const sensor_msgs::ImagePtr msg);

    // returns the total number of images being iterated through for the example data
    int getTotalImageCount();

    void setupFileRead();

    sensor_msgs::ImagePtr readImage(int count_);

    bool fileExists(const std::string& filename);


private:
    ros::NodeHandle nh_;
    ros::Publisher image_pub_;
    int totalImageCount_;
    std::string folderDirectory_;
};

#endif // IMAGE_PROCESSOR_H
