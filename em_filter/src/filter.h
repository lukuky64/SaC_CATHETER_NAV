#ifndef FILTER
#define FILTER

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <Eigen/Geometry>
#include <deque>


struct CallbackData
{
    // Buffers to store recent position and orientation data
    std::deque<Eigen::Vector3d> position_buffer_;
    std::deque<Eigen::Quaterniond> orientation_buffer_;

    // Kalman filter variables for position (x, y, z)
    double x_[3];  // State estimate
    double P_[3] = {1, 1, 1};  // State covariance matrix
    double Q_ = 0.1;  // Process noise covariance
    double R_ = 1;  // Measurement noise covariance
    double K_;  // Kalman gain

    int counter_ = 0;
    bool first_iteration_ = true;
    const size_t buffer_size_ = 3;  // Maximum buffer size for position and orientation

    geometry_msgs::PoseWithCovarianceStamped filtered_pose;  // ROS message to store the filtered pose
    Eigen::Vector3d avg_position;  // Averaged position vector

    Eigen::Vector3d position;  // position vector
    Eigen::Quaterniond orientation;  // orientation quaternion

    Eigen::Quaterniond avg_orientation;  // Averaged orientation quaternion
};


class Filter {
public:
    Filter(ros::NodeHandle &nh);  // Constructor, takes ROS NodeHandle by reference
    ~Filter();  // Destructor

    // Callback function to receive PoseWithCovarianceStamped messages
    void EMCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

    void pointCloudCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    
    // Function to publish filtered pose
    void EMPublish(geometry_msgs::PoseWithCovarianceStamped msg);
    void pointCloudPublish(geometry_msgs::PoseWithCovarianceStamped msg);

private:
    ros::NodeHandle nh_;  // Node handle for ROS operations
    ros::Subscriber EM_sub_;  // Subscriber to receive PoseWithCovarianceStamped messages
    ros::Subscriber point_cloud_sub_;  // Subscriber to receive PoseWithCovarianceStamped messages
    ros::Publisher EM_filtered_pub_;  // Publisher to send out filtered pose messages
    ros::Publisher PC_filtered_pub_; // pose of the point cloud

    CallbackData EM_data;  // Callback data for EM
    CallbackData PC_data;  // Callback data for point cloud

    // Kalman filter function to update estimate and error covariance
    void kalmanFilter(double &estimate, double &error_cov, double measurement, CallbackData data);
};

#endif // FILTER
