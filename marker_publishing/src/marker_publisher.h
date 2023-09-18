#ifndef MARKER_PUBLISHER
#define MARKER_PUBLISHER

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include <tf/transform_broadcaster.h>

class PoseToArrowMarker
{
public:
    PoseToArrowMarker(ros::NodeHandle &nh);
    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

private:
    ros::Subscriber pose_subscriber_;
    ros::Publisher marker_publisher_;
    std::vector<visualization_msgs::Marker> marker_array_;
    int markerLimit_;
};

#endif // MARKER_PUBLISHER
