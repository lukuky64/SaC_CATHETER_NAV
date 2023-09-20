#include "sensor_publisher.h"

// Constructor
sensorPublish::sensorPublish(ros::NodeHandle &nh) : nh_(nh)
{
    image_pub_ = nh_.advertise<sensor_msgs::Image>("image_topic", 10);
    EM_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("em_odometry", 10);
    sensorPublish::setupFileRead();
    refresh_rate = 30;
}

void sensorPublish::setupFileRead()
{
    char buff[FILENAME_MAX];
    getcwd(buff, FILENAME_MAX);
    std::string current_working_dir(buff);
    folderDirectory_ = current_working_dir + "/Data2_Soft_pullback_1/";
    totalDataCount_ = 2002;
}

int sensorPublish::getRefreshRate()
{
    return refresh_rate;
}

bool sensorPublish::fileExists(const std::string &imagePath_)
{
    std::ifstream file(imagePath_);
    return file.good();
}

sensor_msgs::ImagePtr sensorPublish::readImage(int count_)
{
    std::string image_path = folderDirectory_ + "Images/" + std::to_string(count_) + ".jpg";

    if (!fileExists(image_path))
    {
        ROS_WARN("File does not exist: %s, are you in the correct folder?", image_path.c_str());
        return nullptr;
    }

    cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);

    // Convert the image to greyscale
    cv::Mat grey_image;
    cv::cvtColor(image, grey_image, cv::COLOR_BGR2GRAY);

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", grey_image).toImageMsg();
    return msg;
}

geometry_msgs::PoseWithCovarianceStamped sensorPublish::readEM(int count_)
{
    geometry_msgs::PoseWithCovarianceStamped msg;
    std::string em_path = folderDirectory_ + "EM/" + std::to_string(count_) + ".txt";

    if (!fileExists(em_path))
    {
        ROS_WARN("File does not exist: %s, are you in the correct folder?", em_path.c_str());
        msg.header.frame_id = "invalid"; // Using flag to indicate an error
        return msg;
    }

    std::ifstream file(em_path);
    std::string line;
    if (std::getline(file, line))
    {
        std::istringstream ss(line);
        std::string value;
        std::vector<float> values;
        while (std::getline(ss, value, ' '))
        {
            values.push_back(std::stof(value));
        }

        // Assuming the file has 7 values for position (x, y, z) and orientation (x, y, z, w)
        if (values.size() >= 7)
        {
            msg.pose.pose.position.x = values[0];
            msg.pose.pose.position.y = values[1];
            msg.pose.pose.position.z = values[2];
            msg.pose.pose.orientation.x = values[3];
            msg.pose.pose.orientation.y = values[4];
            msg.pose.pose.orientation.z = values[6];
            msg.pose.pose.orientation.w = values[5]; // need to get the ordering correct, but this one should be right
        }

        // sensorPublish::getCovariance(&msg, count_);

        current_time = ros::Time::now();
        msg.header.stamp.sec = current_time.sec;
        msg.header.stamp.nsec = current_time.nsec;
        msg.header.frame_id = "world";
    }
    return msg;
}

void sensorPublish::getCovariance(geometry_msgs::PoseWithCovarianceStamped &msg, int n)
{
    // nothing here yet, need to figure out how to do this correctly
}

int sensorPublish::getTotalDataCount()
{
    return totalDataCount_;
}

// Destructor
sensorPublish::~sensorPublish()
{
    // Destructor code
}

void sensorPublish::imagePublish(const sensor_msgs::ImagePtr msg)
{
    image_pub_.publish(msg);
}

void sensorPublish::EMPublish(geometry_msgs::PoseWithCovarianceStamped msg)
{
    EM_pub_.publish(msg);
}

// Main function
int main(int argc, char **argv)
{
    ros::init(argc, argv, "sensor_publisher");
    ros::NodeHandle nh;
    sensorPublish up(nh);
    ros::Rate loop_rate(up.getRefreshRate());

    int count = 0;
    sensor_msgs::ImagePtr USmsg;
    geometry_msgs::PoseWithCovarianceStamped EMmsg;

    ROS_INFO("Publishing data...");

    while (ros::ok())
    {
        USmsg = up.readImage(count);
        EMmsg = up.readEM(count); // create function

        if ((USmsg != nullptr) || (EMmsg.header.frame_id != "invalid"))
        {
            if (!(USmsg == nullptr))
            {
                ROS_DEBUG_STREAM("Publishing image.........." << count);
                up.imagePublish(USmsg);
            }

            if (!(EMmsg.header.frame_id == "invalid"))
            {
                ROS_DEBUG_STREAM("Publishing EM odometry...." << count);
                up.EMPublish(EMmsg);
            }

            count++;
            ros::spinOnce();
            loop_rate.sleep();
        }

        if (count > up.getTotalDataCount())
        {
            count = 1;
        }
    }

    return 0;
}
