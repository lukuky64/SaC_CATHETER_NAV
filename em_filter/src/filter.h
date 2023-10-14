#ifndef FILTER
#define FILTER

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <Eigen/Geometry>
#include <deque>

class Filter {
public:
    Filter(ros::NodeHandle &nh);
    ~Filter();

    void EMCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    void EMPublish(geometry_msgs::PoseWithCovarianceStamped msg);

private:
    ros::NodeHandle nh_;
    ros::Subscriber EM_sub_;
    ros::Publisher EM_filtered_pub_;
    std::deque<Eigen::Vector3d> position_buffer_;
    std::deque<Eigen::Quaterniond> orientation_buffer_;
    const size_t buffer_size_;
    int counter_;
    geometry_msgs::PoseWithCovarianceStamped filtered_pose;
    Eigen::Vector3d avg_position;
    Eigen::Quaterniond avg_orientation;
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;

    // Kalman filter variables for position (x, y, z)
    double x_[3] = {0, 0, 0};
    double P_[3] = {1, 1, 1};
    double Q_ = 0.1;
    double R_ = 1;
    double K_;

    void kalmanFilter(double& estimate, double& error_cov, double measurement);
};

#endif // FILTER