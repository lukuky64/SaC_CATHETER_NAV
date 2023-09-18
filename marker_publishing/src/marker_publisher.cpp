#include "marker_publisher.h"

PoseToArrowMarker::PoseToArrowMarker(ros::NodeHandle &nh)
{
    pose_subscriber_ = nh.subscribe("/em_odometry", 10, &PoseToArrowMarker::poseCallback, this);
    marker_publisher_ = nh.advertise<visualization_msgs::MarkerArray>("/arrow_marker_array", 10);
    markerLimit_ = 2002;
}

void PoseToArrowMarker::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = msg->header.frame_id;
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::ARROW; // other type: POINTS
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = marker_array_.size();
    marker.pose = msg->pose.pose;
    marker.scale.x = 5.0;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    if (marker_array_.size() > markerLimit_)
    {
        marker_array_.clear();
    }

    marker_array_.push_back(marker);

    visualization_msgs::MarkerArray marker_array_msg;
    marker_array_msg.markers = marker_array_;

    marker_publisher_.publish(marker_array_msg);

    // Publish tf transform for the frame
    static tf::TransformBroadcaster br;
    tf::Transform catheter_tf;
    catheter_tf.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );
    catheter_tf.setRotation(q);
    br.sendTransform(tf::StampedTransform(catheter_tf, ros::Time::now(), "world", "catheter_frame"));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_to_arrow_marker");
    ros::NodeHandle nh;

    PoseToArrowMarker poseToArrowMarker(nh);

    ros::spin();

    return 0;
}
