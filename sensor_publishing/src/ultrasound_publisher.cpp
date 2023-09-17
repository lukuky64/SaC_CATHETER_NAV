#include "ultrasound_publisher.h"

// Constructor
ultrasoundPublish::ultrasoundPublish(ros::NodeHandle &nh) : nh_(nh)
{
    image_pub_ = nh_.advertise<sensor_msgs::Image>("image_topic", 10);
    ultrasoundPublish::setupFileRead();
}

void ultrasoundPublish::setupFileRead()
{
    char buff[FILENAME_MAX];
    getcwd(buff, FILENAME_MAX);
    std::string current_working_dir(buff);
    folderDirectory_ = current_working_dir + "/Data2_Soft_pullback_1/Images/";
    totalImageCount_ = 2002;
}

bool ultrasoundPublish::fileExists(const std::string &imagePath_)
{
    std::ifstream file(imagePath_);
    return file.good();
}

sensor_msgs::ImagePtr ultrasoundPublish::readImage(int count_)
{
    std::string image_path = folderDirectory_ + std::to_string(count_) + ".jpg";
    cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);

    if (!fileExists(image_path))
    {
        ROS_WARN("File does not exist: %s, are you in the correct folder?", image_path.c_str());
        return nullptr;
    }

    // Convert the image to greyscale
    cv::Mat grey_image;
    cv::cvtColor(image, grey_image, cv::COLOR_BGR2GRAY);

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", grey_image).toImageMsg();
    return msg;

}

int ultrasoundPublish::getTotalImageCount()
{
    return totalImageCount_;
}

// Destructor
ultrasoundPublish::~ultrasoundPublish()
{
    // Destructor code
}

void ultrasoundPublish::imagePublish(const sensor_msgs::ImagePtr msg)
{
    image_pub_.publish(msg);
}

// Main function
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ultrasound_publisher");
    ros::NodeHandle nh;
    ultrasoundPublish up(nh);
    ros::Rate loop_rate(30);

    int count = 0;
    sensor_msgs::ImagePtr msg;

    while (ros::ok())
    {
        msg = up.readImage(count);

        if (!(msg == nullptr))
        {
            ROS_INFO("Publishing image %d", count);
            up.imagePublish(msg);
            count++;
            ros::spinOnce();
            loop_rate.sleep();
        }

        if (count > up.getTotalImageCount())
        {
            count = 1;
        }
    }

    return 0;
}
