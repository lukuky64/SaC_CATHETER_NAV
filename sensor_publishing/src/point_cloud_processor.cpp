#include "point_cloud_processor.h"
#include <pcl_conversions/pcl_conversions.h>

PointCloudProcessor::PointCloudProcessor()
{
  cloud_.height = 1;      // unorganized point cloud
  cloud_.is_dense = true; // all points are finite (not inf, nan, etc.)
}

PointCloudProcessor::~PointCloudProcessor()
{
  //
}

std::vector<Eigen::Vector3d> PointCloudProcessor::projectContours(std::vector<Eigen::Vector2d> contours, geometry_msgs::PoseWithCovarianceStamped pose)
{
  std::vector<Eigen::Vector3d> projectedContours_;
  Eigen::Vector3d position(pose.pose.pose.position.x, pose.pose.pose.position.y, pose.pose.pose.position.z);
  Eigen::Quaterniond quaternion(pose.pose.pose.orientation.w, pose.pose.pose.orientation.x, pose.pose.pose.orientation.y, pose.pose.pose.orientation.z);
  Eigen::Matrix3d rotationMatrix = quaternion.toRotationMatrix();

  for (size_t i = 0; i < contours.size(); i++)
  {
    Eigen::Vector3d point(contours[i].x(), contours[i].y(), 0.0);
    Eigen::Vector3d projectedPoint = rotationMatrix * point + position;
    projectedContours_.push_back(projectedPoint);
  }

  return projectedContours_;
}

void PointCloudProcessor::saveProjectedContoursToFile(const std::vector<Eigen::Vector3d> &projectedContours_, const std::string &filename)
{
  // Create an output file stream
  std::ofstream outfile(filename, std::ios::app);

  // Check if the file is open
  if (!outfile.is_open())
  {
    std::cerr << "Failed to open file: " << filename << std::endl;
    return;
  }

  // Write each vector to the file
  for (const auto &vec : projectedContours_)
  {
    outfile << vec[0] << " " << vec[1] << " " << vec[2] << std::endl;
  }

  // Close the file
  outfile.close();
}

std::string PointCloudProcessor::vectorToString(const std::vector<Eigen::Vector2d> &vec)
{
  std::ostringstream oss;
  for (const auto &v : vec)
  {
    oss << "SIZE[" << vec.size() << "] - " << "(" << v[0] << ", " << v[1] << ") ";
  }
  return oss.str();
}

sensor_msgs::PointCloud2 PointCloudProcessor::createPointCloud(std::vector<Eigen::Vector2d> contours, geometry_msgs::PoseWithCovarianceStamped pose)
{
  // Project the incoming contours to the global frame
  std::vector<Eigen::Vector3d> projectedContours_ = projectContours(contours, pose);

  // DEBUGGING
  //std::string allContours = vectorToString(contours);
  //ROS_INFO("All Projected Contours: %s", allContours.c_str());

  // saveProjectedContoursToFile(projectedContours_, "projected_contours.txt");

  // Determine the starting index for new points in cloud_
  size_t startIndex = cloud_.points.size();

  // Resize the cloud to hold the new points
  cloud_.width += projectedContours_.size();
  cloud_.points.resize(cloud_.width);

  // Insert the new points into cloud_
  for (size_t i = 0; i < projectedContours_.size(); ++i)
  {
    cloud_.points[startIndex + i].x = projectedContours_[i].x();
    cloud_.points[startIndex + i].y = projectedContours_[i].y();
    cloud_.points[startIndex + i].z = projectedContours_[i].z();
  }

  // Convert to sensor_msgs::PointCloud2
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(cloud_, output);

  // Set the frame ID
  output.header.frame_id = "world";

  return output;
}
