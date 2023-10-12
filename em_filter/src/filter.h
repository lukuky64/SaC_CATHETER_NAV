#ifndef FILTER
#define FILTER

// headers
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class Filter
{
public:
    // Constructor
    Filter(ros::NodeHandle &nh);

    // Destructor
    ~Filter();

    void EMCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

private:
    ros::NodeHandle nh_;     // ROS NodeHandle for managing ROS-related operations
    ros::Subscriber EM_sub_; // ROS Publisher for EM PoseWithCovarianceStamped messages
};

#endif // FILTER