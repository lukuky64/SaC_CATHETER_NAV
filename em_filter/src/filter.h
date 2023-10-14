#ifndef FILTER
#define FILTER

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <Eigen/Geometry>
#include <deque>

class Filter {
public:
    Filter(ros::NodeHandle &nh);  // Constructor, takes ROS NodeHandle by reference
    ~Filter();  // Destructor

    // Callback function to receive PoseWithCovarianceStamped messages
    void EMCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    
    // Function to publish filtered pose
    void EMPublish(geometry_msgs::PoseWithCovarianceStamped msg);

private:
    ros::NodeHandle nh_;  // Node handle for ROS operations
    ros::Subscriber EM_sub_;  // Subscriber to receive PoseWithCovarianceStamped messages
    ros::Publisher EM_filtered_pub_;  // Publisher to send out filtered pose messages

    // Buffers to store recent position and orientation data
    std::deque<Eigen::Vector3d> position_buffer_;
    std::deque<Eigen::Quaterniond> orientation_buffer_;
    
    const size_t buffer_size_;  // Maximum buffer size for position and orientation
    int counter_;  // Counter to keep track of received messages
    
    geometry_msgs::PoseWithCovarianceStamped filtered_pose;  // ROS message to store the filtered pose
    Eigen::Vector3d avg_position;  // Averaged position vector
    Eigen::Quaterniond avg_orientation;  // Averaged orientation quaternion
    
    Eigen::Vector3d position;  // Current position vector
    Eigen::Quaterniond orientation;  // Current orientation quaternion

    // Kalman filter variables for position (x, y, z)
    double x_[3];  // State estimate
    double P_[3] = {1, 1, 1};  // State covariance matrix
    double Q_ = 0.1;  // Process noise covariance
    double R_ = 1;  // Measurement noise covariance
    double K_;  // Kalman gain
    
    bool first_iteration_;  // Flag to identify the first iteration

    // Kalman filter function to update estimate and error covariance
    void kalmanFilter(double& estimate, double& error_cov, double measurement);
};

#endif // FILTER
