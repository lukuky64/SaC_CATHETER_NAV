#include "filter.h"

Filter::Filter(ros::NodeHandle &nh) : nh_(nh), buffer_size_(3), counter_(0), first_iteration_(true)
{
    EM_sub_ = nh_.subscribe("/em_odometry", 10, &Filter::EMCallback, this);
    EM_filtered_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/em_filtered_odometry", 10, true); // latching enabled
}

Filter::~Filter()
{
    // Destructor
}

void Filter::kalmanFilter(double &estimate, double &error_cov, double measurement)
{
    // Prediction
    error_cov += Q_;

    // Update
    K_ = error_cov / (error_cov + R_);
    estimate += K_ * (measurement - estimate);
    error_cov *= (1 - K_);
}

void Filter::EMCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{

    position << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    orientation.w() = msg->pose.pose.orientation.w;
    orientation.x() = msg->pose.pose.orientation.x;
    orientation.y() = msg->pose.pose.orientation.y;
    orientation.z() = msg->pose.pose.orientation.z;

    // Add to buffers
    position_buffer_.push_back(position);
    orientation_buffer_.push_back(orientation);

    // Remove oldest data if buffer is full
    if (position_buffer_.size() > buffer_size_)
    {
        position_buffer_.pop_front();
        orientation_buffer_.pop_front();
    }

    counter_++;
    if (counter_ % buffer_size_ == 0)
    {
        // Average position and orientation
        avg_position = Eigen::Vector3d::Zero();
        avg_orientation.w() = 0;
        avg_orientation.x() = 0;
        avg_orientation.y() = 0;
        avg_orientation.z() = 0;

        for (const auto &pos : position_buffer_)
        {
            avg_position += pos;
        }
        avg_position /= buffer_size_;

        for (const auto &ori : orientation_buffer_)
        {
            avg_orientation.coeffs() += ori.coeffs();
        }
        avg_orientation.coeffs() /= buffer_size_;
        avg_orientation.normalize();


        if (first_iteration_)
        {
            x_[0] = msg->pose.pose.position.x;
            x_[1] = msg->pose.pose.position.y;
            x_[2] = msg->pose.pose.position.z;
            first_iteration_ = false;
        }

        // Apply Kalman filter to averaged position
        for (int i = 0; i < 3; ++i)
        {
            kalmanFilter(x_[i], P_[i], avg_position[i]);
        }

        filtered_pose.pose.pose.position.x = x_[0];
        filtered_pose.pose.pose.position.y = x_[1];
        filtered_pose.pose.pose.position.z = x_[2];
        filtered_pose.pose.pose.orientation.w = avg_orientation.w();
        filtered_pose.pose.pose.orientation.x = avg_orientation.x();
        filtered_pose.pose.pose.orientation.y = avg_orientation.y();
        filtered_pose.pose.pose.orientation.z = avg_orientation.z();

        filtered_pose.header.stamp = ros::Time::now();
        filtered_pose.header.frame_id = "world";

        EMPublish(filtered_pose);
    }
}

void Filter::EMPublish(geometry_msgs::PoseWithCovarianceStamped msg)
{
    EM_filtered_pub_.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "filter_publisher");
    ros::NodeHandle nh;

    Filter filter_(nh);

    while (ros::ok())
    {
        ros::spinOnce();
    }

    return 0;
}
