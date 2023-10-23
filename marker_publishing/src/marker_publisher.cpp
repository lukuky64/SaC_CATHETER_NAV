#include "marker_publisher.h"

PoseToMarker::PoseToMarker(ros::NodeHandle &nh)
{
    //pc_pose_subscriber_ = nh.subscribe("/point_cloud_filtered_odometry", 10, &PoseToMarker::pcPoseCallback, this);
    pose_subscriber_ = nh.subscribe("/em_filtered_odometry", 10, &PoseToMarker::poseCallback, this);
    prediction_subscriber_ = nh.subscribe("/prediction", 10, &PoseToMarker::predictionCallback, this);
    arrow_marker_publisher_ = nh.advertise<visualization_msgs::Marker>("/arrow_marker", 10, true);
    pose_line_strip_marker_publisher_ = nh.advertise<visualization_msgs::Marker>("/line_marker_array", 10, true);
    predict_line_strip_marker_publisher_ = nh.advertise<visualization_msgs::Marker>("/predict_marker_array", 10, true);
    sphere_marker_publisher_ = nh.advertise<visualization_msgs::Marker>("/catheter_marker", 10, true);
    markerLimit_ = 2002;

    ROS_INFO("Starting publishing markers...");
}

void PoseToMarker::predictionCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    PoseToMarker::createPredictionLineMarker(msg);
}

void PoseToMarker::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    PoseToMarker::createArrowMarker(msg);
    PoseToMarker::createPoseLineMarker(msg);
    PoseToMarker::createCatheterMarker(msg);

    // Publish tf transform for the frame
    static tf::TransformBroadcaster br;
    tf::Transform catheter_tf;
    catheter_tf.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

    catheter_tf.setRotation(q);
    br.sendTransform(tf::StampedTransform(catheter_tf, ros::Time::now(), "world", "catheter_frame"));
}

void PoseToMarker::createCatheterMarker(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    // Initialize and configure a spherical marker
    visualization_msgs::Marker sphere_marker;
    sphere_marker.header.frame_id = msg->header.frame_id;
    sphere_marker.header.stamp = ros::Time::now();
    sphere_marker.type = visualization_msgs::Marker::SPHERE;
    sphere_marker.action = visualization_msgs::Marker::ADD;
    sphere_marker.id = 0;

    // Set the pose of the marker to the incoming message's pose
    sphere_marker.pose.position.x = msg->pose.pose.position.x;
    sphere_marker.pose.position.y = msg->pose.pose.position.y;
    sphere_marker.pose.position.z = msg->pose.pose.position.z;

    // Initialize the quaternion (Identity Quaternion)
    sphere_marker.pose.orientation.x = 0.0;
    sphere_marker.pose.orientation.y = 0.0;
    sphere_marker.pose.orientation.z = 0.0;
    sphere_marker.pose.orientation.w = 1.0;

    // Set other properties of the marker
    sphere_marker.scale.x = 0.3;
    sphere_marker.scale.y = 0.3;
    sphere_marker.scale.z = 0.3;
    sphere_marker.color.r = 1.0;
    sphere_marker.color.a = 1.0; // Opacity

    // Publish the marker
    sphere_marker_publisher_.publish(sphere_marker);
}

void PoseToMarker::createArrowMarker(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    // arrow marker
    visualization_msgs::Marker marker;
    marker.header.frame_id = msg->header.frame_id;
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::ARROW; // other type: POINTS or LINE_STRIP or ARROW
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = arrow_marker_array_.size();

    marker.pose = msg->pose.pose;

    marker.scale.x = 1.4;
    marker.scale.y = 0.075;
    marker.scale.z = 0.075;
    marker.color.a = 0.75; // Opacity
    marker.color.r = 1.0;

    arrow_marker_publisher_.publish(marker);
}

void PoseToMarker::createPoseLineMarker(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    // LINE_STRIP Marker
    pose_line_strip_marker.header.frame_id = msg->header.frame_id;
    pose_line_strip_marker.header.stamp = ros::Time::now();
    pose_line_strip_marker.ns = "line_strip";
    pose_line_strip_marker.action = visualization_msgs::Marker::ADD;
    pose_line_strip_marker.pose.orientation.w = 1.0;
    pose_line_strip_marker.id = 0;
    pose_line_strip_marker.type = visualization_msgs::Marker::LINE_STRIP;

    // LINE_STRIP settings
    pose_line_strip_marker.scale.x = 0.08;
    pose_line_strip_marker.color.r = 1.0; // setting colour to red
    pose_line_strip_marker.color.a = 1.0; // Opacity

    geometry_msgs::Point point;
    point.x = msg->pose.pose.position.x;
    point.y = msg->pose.pose.position.y;
    point.z = msg->pose.pose.position.z;

    pose_line_strip_marker.points.push_back(point);

    pose_line_strip_marker_publisher_.publish(pose_line_strip_marker);
}

void PoseToMarker::createPredictionLineMarker(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    // LINE_STRIP Marker
    predict_line_strip_marker.header.frame_id = "world";
    predict_line_strip_marker.header.stamp = ros::Time::now();
    predict_line_strip_marker.ns = "prediction_line_strip";
    predict_line_strip_marker.action = visualization_msgs::Marker::ADD;
    predict_line_strip_marker.pose.orientation.w = 1.0;
    predict_line_strip_marker.id = 0;
    predict_line_strip_marker.type = visualization_msgs::Marker::LINE_STRIP;

    // LINE_STRIP settings
    predict_line_strip_marker.scale.x = 0.08;
    predict_line_strip_marker.color.g = 1.0; // setting colour to green
    predict_line_strip_marker.color.a = 1.0; // Opacity

    geometry_msgs::Point point;
    point.x = msg->pose.pose.position.x;
    point.y = msg->pose.pose.position.y;
    point.z = msg->pose.pose.position.z;

    predict_line_strip_marker.points.push_back(point);

    predict_line_strip_marker_publisher_.publish(predict_line_strip_marker);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "markers");
    ros::NodeHandle nh;

    PoseToMarker PoseToMarker(nh);

    ros::spin();

    return 0;
}
