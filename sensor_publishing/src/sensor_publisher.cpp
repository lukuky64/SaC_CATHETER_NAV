#include "sensor_publisher.h"

// Constructor
sensorPublish::sensorPublish(ros::NodeHandle &nh, std::string baseFile_) : nh_(nh), paused_(true), previous_position_(0.0, 0.0, 0.0), current_quaternion_(0.0, 0.0, 0.0, 1.0)
{
    image_pub_ = nh_.advertise<sensor_msgs::Image>("raw_image_topic", 10, true);
    EM_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("em_odometry", 10, true);
    PCP_Pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("point_cloud_odometry", 10, true);
    pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/point_cloud_topic", 10, true);
    sensorPublish::setupFileRead(baseFile_);
    refresh_rate = 30;
}

void sensorPublish::setupFileRead(std::string baseFile_)
{
    folderDirectory_ = baseFile_ + "/Data2_Soft_pullback_1/";
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

bool sensorPublish::setState(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    paused_ = req.data;
    res.success = true;
    res.message = "Successfully set state";
    return true;
}

sensor_msgs::PointCloud2 sensorPublish::readContour(int count_, geometry_msgs::PoseWithCovarianceStamped pose)
{
    sensor_msgs::PointCloud2 msg;
    std::string contour_path = folderDirectory_ + "Contours/" + std::to_string(count_) + ".txt";

    std::vector<Eigen::Vector2d> contours;

    if (!fileExists(contour_path))
    {
        ROS_WARN("File does not exist: %s, are you in the correct folder?", contour_path.c_str());
        msg.header.frame_id = "invalid"; // Using flag to indicate an error
        return msg;
    }

    std::ifstream file(contour_path);
    std::string line;
    if (std::getline(file, line))
    {
        std::istringstream ss(line);
        std::string value;
        std::vector<float> values;

        while (std::getline(ss, value, ' '))
        {
            float x_val = std::stof(value);
            std::getline(ss, value, ' ');
            float y_val = std::stof(value);
            contours.push_back(Eigen::Vector2d(x_val, y_val));
        }

        msg = pCloudProcessor.createPointCloud(contours, pose);
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "world";
    }
    return msg;
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
    // cv::Mat grey_image;
    // cv::cvtColor(image, grey_image, cv::COLOR_BGR2GRAY);

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    return msg;
}

geometry_msgs::PoseWithCovarianceStamped sensorPublish::createPCPose(geometry_msgs::PoseWithCovarianceStamped msg)
{
    geometry_msgs::PoseWithCovarianceStamped PC_pose = msg;

    current_position_.x() = msg.pose.pose.position.x;
    current_position_.y() = msg.pose.pose.position.y;
    current_position_.z() = msg.pose.pose.position.z;

    double dist_ = euclideanDistance(current_position_, previous_position_);

    // to create more reliable quaternion data, only update the quaternion if the distance between the points is greater than 10mm
    if (dist_ > 1)
    {
        current_quaternion_ = calculateRotation(previous_position_, current_position_);
        previous_position_ = current_position_;
    }

    PC_pose.pose.pose.orientation.x = current_quaternion_.x();
    PC_pose.pose.pose.orientation.y = current_quaternion_.y();
    PC_pose.pose.pose.orientation.z = current_quaternion_.z();
    PC_pose.pose.pose.orientation.w = current_quaternion_.w();

    return PC_pose;
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

            // !!! Currently, given quaternion data is not useful so we will need to interpolate between points instead
            // current_position_ << static_cast<double>(values[0]), static_cast<double>(values[1]), static_cast<double>(values[2]);
            // current_quaternion_ = calculateRotation(current_position_, previous_position_);
            // previous_position_ = current_position_;

            // msg.pose.pose.orientation.x = current_quaternion_.x();
            // msg.pose.pose.orientation.y = current_quaternion_.y();
            // msg.pose.pose.orientation.z = current_quaternion_.z();
            // msg.pose.pose.orientation.w = current_quaternion_.w();

            // tf2::Quaternion original_q(values[3], values[4], values[5], values[6]);
            // tf2::Quaternion offset_q(0, 0, 0, 1);

            // tf2::Quaternion new_q = offset_q * original_q;
            // new_q.normalize();

            msg.pose.pose.orientation.x = values[3]; // new_q.x();
            msg.pose.pose.orientation.y = values[4]; // new_q.y();
            msg.pose.pose.orientation.z = values[5]; // new_q.z();
            msg.pose.pose.orientation.w = values[6]; // new_q.w();

            // msg.pose.pose.orientation.x = values[3];
            // msg.pose.pose.orientation.y = values[4] * (0.7071068);
            // msg.pose.pose.orientation.z = values[6];
            // msg.pose.pose.orientation.w = values[5] * (0.7071068); // need to get the ordering correct, but this one should be right
        }

        // sensorPublish::getCovariance(&msg, count_);
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "world";
    }
    return msg;
}

double sensorPublish::euclideanDistance(const Eigen::Vector3d &vec1, const Eigen::Vector3d &vec2)
{
    return (vec1 - vec2).norm();
}

void sensorPublish::getCovariance(geometry_msgs::PoseWithCovarianceStamped &msg, int n)
{
    // nothing here yet, need to figure out how to do this correctly
}

int sensorPublish::getTotalDataCount()
{
    return totalDataCount_;
}

bool sensorPublish::getPaused_state()
{
    return paused_;
}

void sensorPublish::setPaused_state(bool state)
{
    paused_ = state;
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

void sensorPublish::PCPPublish(geometry_msgs::PoseWithCovarianceStamped msg)
{
    PCP_Pub_.publish(msg);
}

void sensorPublish::CloudPublish(sensor_msgs::PointCloud2 msg)
{
    pcl_pub_.publish(msg);
}

Eigen::Quaterniond sensorPublish::calculateRotation(const Eigen::Vector3d &point1, const Eigen::Vector3d &point2)
{
    // Calculate unit vectors for the original and target directions
    Eigen::Vector3d originalDirection = Eigen::Vector3d(1.0, 0.0, 0.0); // Assuming original direction is along the x-axis
    Eigen::Vector3d targetDirection = (point2 - point1).normalized();

    // Calculate rotation axis and angle
    Eigen::Vector3d rotationAxis = originalDirection.cross(targetDirection).normalized();
    double angle = std::acos(originalDirection.dot(targetDirection));

    // Create quaternion from axis and angle
    Eigen::Quaterniond quaternion(Eigen::AngleAxisd(angle, rotationAxis));

    return quaternion;
}

// Main function
int main(int argc, char *argv[])
{

    ros::init(argc, argv, "sensor_publisher");
    ros::NodeHandle nh;

    std::string folder_;
    if (nh.getParam("folder_", folder_))
    {
        ROS_INFO("Got path: %s", folder_.c_str());
    }
    else
    {
        ROS_ERROR("Failed to get parameter 'my_path'");
        return -1;
    }

    sensorPublish up(nh, folder_);

    ros::ServiceServer paused_service = nh.advertiseService("set_paused", &sensorPublish::setState, &up);

    ros::Rate loop_rate(up.getRefreshRate());

    int count = 0;
    sensor_msgs::ImagePtr USmsg;
    geometry_msgs::PoseWithCovarianceStamped EMmsg;
    geometry_msgs::PoseWithCovarianceStamped PCmsg;
    sensor_msgs::PointCloud2 Cloudmsg;

    std::this_thread::sleep_for(std::chrono::seconds(5)); // wait a bit before publishing so RVIZ and other programs can load

    ROS_INFO("Publishing data...");

    while (ros::ok())
    {
        USmsg = up.readImage(count);
        PCmsg = up.readEM(count);
        EMmsg = up.createPCPose(PCmsg);
        Cloudmsg = up.readContour(count, PCmsg);

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

            if (!(PCmsg.header.frame_id == "invalid"))
            {
                ROS_DEBUG_STREAM("Publishing PC odometry...." << count);
                up.PCPPublish(PCmsg);
            }

            if (!(Cloudmsg.header.frame_id == "invalid"))
            {
                ROS_DEBUG_STREAM("Publishing PC odometry...." << count);
                up.CloudPublish(Cloudmsg);
            }

            count++;
            ros::spinOnce();
            loop_rate.sleep();
        }

        if (count == up.getTotalDataCount())
        {
            up.setPaused_state(true);
            // count = 1; // to loop back
        }

        if (count > 1)
        {
            while (up.getPaused_state() == true)
            {
                // wait here while publisher is paused
                ros::spinOnce(); // Process incoming messages and service calls
            }
        }
    }

    return 0;
}
