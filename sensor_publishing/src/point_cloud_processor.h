#ifndef POINT_CLOUD_PROCESSOR_H
#define POINT_CLOUD_PROCESSOR_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class PointCloudProcessor
{
public:
  PointCloudProcessor();
  ~PointCloudProcessor();

  // creates a 3D point cloud from a vector of contours and a pose
  sensor_msgs::PointCloud2 createPointCloud(std::vector<Eigen::Vector2d> contours, geometry_msgs::PoseWithCovarianceStamped pose);

  // projects a vector of 2D contours to 3D space
  std::vector<Eigen::Vector3d> projectContours(std::vector<Eigen::Vector2d> contours, geometry_msgs::PoseWithCovarianceStamped pose);

private:
  ros::NodeHandle nh_;

  // 3D point cloud
  pcl::PointCloud<pcl::PointXYZ> cloud_;
};

#endif // POINT_CLOUD_PROCESSOR_H