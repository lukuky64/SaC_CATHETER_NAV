#ifndef MARKER_PUBLISHER
#define MARKER_PUBLISHER

#include <mutex>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include <tf/transform_broadcaster.h>
#include <Eigen/Geometry>


class PoseToMarker
{
public:
    PoseToMarker(ros::NodeHandle &nh);
    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    void predictionCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    void createArrowMarker(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    void createPoseLineMarker(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    void createPredictionLineMarker(Eigen::Vector3d &msg);
    void createPredictionLineMarker(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    void createCatheterMarker(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    void pcPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

private:
    ros::Subscriber pose_subscriber_;
    ros::Subscriber pc_pose_subscriber_;
    ros::Subscriber prediction_subscriber_;
    ros::Publisher arrow_marker_publisher_;
    ros::Publisher pose_line_strip_marker_publisher_;
    ros::Publisher predict_line_strip_marker_publisher_;
    ros::Publisher sphere_marker_publisher_;
    std::vector<visualization_msgs::Marker> arrow_marker_array_;
    std::vector<visualization_msgs::Marker> line_marker_array_;
    visualization_msgs::Marker pose_line_strip_marker;
    visualization_msgs::Marker predict_line_strip_marker;
    int markerLimit_;
    
    geometry_msgs::PoseWithCovarianceStamped::ConstPtr latestPose;
    std::mutex poseMutex;
};

#endif // MARKER_PUBLISHER
