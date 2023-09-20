#ifndef MARKER_PUBLISHER
#define MARKER_PUBLISHER

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include <tf/transform_broadcaster.h>

class PoseToMarker
{
public:
    PoseToMarker(ros::NodeHandle &nh);
    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    void createArrowMarker(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    void createLineMarker(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    void createCatheterMarker(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

private:
    ros::Subscriber pose_subscriber_;
    ros::Publisher arrow_marker_publisher_;
    ros::Publisher line_strip_marker_publisher_;
    ros::Publisher sphere_marker_publisher_;
    std::vector<visualization_msgs::Marker> arrow_marker_array_;
    std::vector<visualization_msgs::Marker> line_marker_array_;
    visualization_msgs::Marker line_strip_marker;
    int markerLimit_;
};

#endif // MARKER_PUBLISHER
