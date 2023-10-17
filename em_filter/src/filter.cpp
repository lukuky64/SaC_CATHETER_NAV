#include "filter.h"

Filter::Filter(ros::NodeHandle &nh) : nh_(nh)
{
    EM_sub_ = nh_.subscribe("/em_odometry", 10, &Filter::EMCallback, this);
    point_cloud_sub_ = nh_.subscribe("/point_cloud_odometry", 10, &Filter::pointCloudCallback, this);
    
    EM_filtered_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/em_filtered_odometry", 10, true); // latching enabled
    PC_filtered_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/point_cloud_filtered_odometry", 10, true); // latching enabled
}

Filter::~Filter()
{
    // Destructor
}

void Filter::kalmanFilter(double &estimate, double &error_cov, double measurement, CallbackData data)
{
    // Prediction
    error_cov += data.Q_;

    // Update
    data.K_ = error_cov / (error_cov + data.R_);
    estimate += data.K_ * (measurement - estimate);
    error_cov *= (1 - data.K_);
}

void Filter::pointCloudCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    PC_data.position << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    PC_data.orientation.w() = msg->pose.pose.orientation.w;
    PC_data.orientation.x() = msg->pose.pose.orientation.x;
    PC_data.orientation.y() = msg->pose.pose.orientation.y;
    PC_data.orientation.z() = msg->pose.pose.orientation.z;

    // Add to buffers
    PC_data.position_buffer_.push_back(PC_data.position);
    PC_data.orientation_buffer_.push_back(PC_data.orientation);

    // Remove oldest data if buffer is full
    if (PC_data.position_buffer_.size() > PC_data.buffer_size_)
    {
        PC_data.position_buffer_.pop_front();
        PC_data.orientation_buffer_.pop_front();
    }

    PC_data.counter_++;
    if (PC_data.counter_ % PC_data.buffer_size_ == 0)
    {
        // Average position and orientation
        PC_data.avg_position = Eigen::Vector3d::Zero();
        PC_data.avg_orientation.w() = 0;
        PC_data.avg_orientation.x() = 0;
        PC_data.avg_orientation.y() = 0;
        PC_data.avg_orientation.z() = 0;

        for (const auto &pos : PC_data.position_buffer_)
        {
            PC_data.avg_position += pos;
        }
        PC_data.avg_position /= PC_data.buffer_size_;

        for (const auto &ori : PC_data.orientation_buffer_)
        {
            PC_data.avg_orientation.coeffs() += ori.coeffs();
        }
        PC_data.avg_orientation.coeffs() /= PC_data.buffer_size_;
        PC_data.avg_orientation.normalize();


        if (PC_data.first_iteration_)
        {
            PC_data.x_[0] = msg->pose.pose.position.x;
            PC_data.x_[1] = msg->pose.pose.position.y;
            PC_data.x_[2] = msg->pose.pose.position.z;
            PC_data.first_iteration_ = false;
        }

        // Apply Kalman filter to averaged position
        for (int i = 0; i < 3; ++i)
        {
            kalmanFilter(PC_data.x_[i], PC_data.P_[i], PC_data.avg_position[i], PC_data);
        }

        PC_data.filtered_pose.pose.pose.position.x = PC_data.x_[0];
        PC_data.filtered_pose.pose.pose.position.y = PC_data.x_[1];
        PC_data.filtered_pose.pose.pose.position.z = PC_data.x_[2];
        PC_data.filtered_pose.pose.pose.orientation.w = PC_data.avg_orientation.w();
        PC_data.filtered_pose.pose.pose.orientation.x = PC_data.avg_orientation.x();
        PC_data.filtered_pose.pose.pose.orientation.y = PC_data.avg_orientation.y();
        PC_data.filtered_pose.pose.pose.orientation.z = PC_data.avg_orientation.z();

        PC_data.filtered_pose.header.stamp = ros::Time::now();
        PC_data.filtered_pose.header.frame_id = "world";

        pointCloudPublish(PC_data.filtered_pose);
    }
}

void Filter::pointCloudPublish(geometry_msgs::PoseWithCovarianceStamped msg)
{
    PC_filtered_pub_.publish(msg);
}

void Filter::EMCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{

    EM_data.position << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    EM_data.orientation.w() = msg->pose.pose.orientation.w;
    EM_data.orientation.x() = msg->pose.pose.orientation.x;
    EM_data.orientation.y() = msg->pose.pose.orientation.y;
    EM_data.orientation.z() = msg->pose.pose.orientation.z;

    // Add to buffers
    EM_data.position_buffer_.push_back(EM_data.position);
    EM_data.orientation_buffer_.push_back(EM_data.orientation);

    // Remove oldest data if buffer is full
    if (EM_data.position_buffer_.size() > EM_data.buffer_size_)
    {
        EM_data.position_buffer_.pop_front();
        EM_data.orientation_buffer_.pop_front();
    }

    EM_data.counter_++;
    if (EM_data.counter_ % EM_data.buffer_size_ == 0)
    {
        // Average position and orientation
        EM_data.avg_position = Eigen::Vector3d::Zero();
        EM_data.avg_orientation.w() = 0;
        EM_data.avg_orientation.x() = 0;
        EM_data.avg_orientation.y() = 0;
        EM_data.avg_orientation.z() = 0;

        for (const auto &pos : EM_data.position_buffer_)
        {
            EM_data.avg_position += pos;
        }
        EM_data.avg_position /= EM_data.buffer_size_;

        for (const auto &ori : EM_data.orientation_buffer_)
        {
            EM_data.avg_orientation.coeffs() += ori.coeffs();
        }
        EM_data.avg_orientation.coeffs() /= EM_data.buffer_size_;
        EM_data.avg_orientation.normalize();


        if (EM_data.first_iteration_)
        {
            EM_data.x_[0] = msg->pose.pose.position.x;
            EM_data.x_[1] = msg->pose.pose.position.y;
            EM_data.x_[2] = msg->pose.pose.position.z;
            EM_data.first_iteration_ = false;
        }

        // Apply Kalman filter to averaged position
        for (int i = 0; i < 3; ++i)
        {
            kalmanFilter(EM_data.x_[i], EM_data.P_[i], EM_data.avg_position[i], EM_data);
        }

        EM_data.filtered_pose.pose.pose.position.x = EM_data.x_[0];
        EM_data.filtered_pose.pose.pose.position.y = EM_data.x_[1];
        EM_data.filtered_pose.pose.pose.position.z = EM_data.x_[2];
        EM_data.filtered_pose.pose.pose.orientation.w = EM_data.avg_orientation.w();
        EM_data.filtered_pose.pose.pose.orientation.x = EM_data.avg_orientation.x();
        EM_data.filtered_pose.pose.pose.orientation.y = EM_data.avg_orientation.y();
        EM_data.filtered_pose.pose.pose.orientation.z = EM_data.avg_orientation.z();

        EM_data.filtered_pose.header.stamp = ros::Time::now();
        EM_data.filtered_pose.header.frame_id = "world";

        EMPublish(EM_data.filtered_pose);
    }
}

void Filter::EMPublish(geometry_msgs::PoseWithCovarianceStamped msg)
{
    EM_filtered_pub_.publish(msg);
}

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "filter_publisher");
//     ros::NodeHandle nh;

//     Filter filter_(nh);

//     while (ros::ok())
//     {
//         ros::spinOnce();
//     }

//     return 0;
// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "filter_publisher");
    ros::NodeHandle nh;

    Filter filter_(nh);

    // Create a multi-threaded spinner with 2 threads
    ros::MultiThreadedSpinner spinner(2);

    // Starting the spinner to handle incoming messages
    spinner.spin();

    return 0;
}
