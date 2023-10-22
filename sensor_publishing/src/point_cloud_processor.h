#ifndef POINT_CLOUD_PROCESSOR_H
#define POINT_CLOUD_PROCESSOR_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Core>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>

// debugging headers
#include <iostream>
#include <fstream>
#include <sstream>

class PointCloudProcessor
{
public:
  PointCloudProcessor();
  ~PointCloudProcessor();

  // creates a 3D point cloud from a vector of contours and a pose
  sensor_msgs::PointCloud2 createPointCloud(std::vector<Eigen::Vector2d> contours, geometry_msgs::PoseWithCovarianceStamped pose);

  // projects a vector of 2D contours to 3D space
  std::vector<Eigen::Vector3d> projectContours(std::vector<Eigen::Vector2d> contours, geometry_msgs::PoseWithCovarianceStamped pose);

  void saveProjectedContoursToFile(const std::vector<Eigen::Vector3d>& projectedContours_, const std::string& filename);

  std::string vectorToString(const std::vector<Eigen::Vector2d>& vec);

  void applyVoxelFilterAndColour(Eigen::Vector3d currentPose); // applies a voxel filter to the point cloud

  //pcl::PolygonMesh createMesh(); // creates a mesh from the filtered point cloud

private:
  ros::NodeHandle nh_;

  // 3D point cloud
  pcl::PointCloud<pcl::PointXYZ> cloud_;
  pcl::PointCloud<pcl::PointXYZRGB> filteredCloud; // filtered point cloud with color representing proximity to the catheter
  pcl::VoxelGrid<pcl::PointXYZ> voxelFilter;

  size_t lastFilter;  // Index of the last filtered point
  size_t batchSize;  // Number of points to filter in each batch

  Eigen::Vector3d previousPose; // Stores the previous pose to check if the catheter has moved

};

#endif // POINT_CLOUD_PROCESSOR_H