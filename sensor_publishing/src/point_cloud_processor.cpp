#include "point_cloud_processor.h"
#include <pcl_conversions/pcl_conversions.h>

PointCloudProcessor::PointCloudProcessor()
{
  cloud_.height = 1;      // unorganized point cloud
  cloud_.is_dense = true; // all points are finite (not inf, nan, etc.)

  lastFilter = 0;                   // Index of the last filtered point
  batchSize = 350;                  // Number of points to filter in each batch
  previousPose << 5000, 5000, 5000; // large value to ensure first point is added
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
    oss << "SIZE[" << vec.size() << "] - "
        << "(" << v[0] << ", " << v[1] << ") ";
  }
  return oss.str();
}

void PointCloudProcessor::applyVoxelFilterAndColour(Eigen::Vector3d currentPose)
{
  // Check if we have enough new points for a batch
  if (cloud_.points.size() - lastFilter >= batchSize)
  {
    // Create a new point cloud to hold the batch of points to filter
    pcl::PointCloud<pcl::PointXYZ> batchCloud;

    // Populate 'batchCloud' with the new points
    for (size_t i = lastFilter; i < lastFilter + batchSize; ++i)
    {
      batchCloud.points.push_back(cloud_.points[i]);
    }

    // Set the input cloud for filtering
    voxelFilter.setInputCloud(batchCloud.makeShared());

    // Set the leaf size (size of the voxel)
    voxelFilter.setLeafSize(2.0f, 2.0f, 2.0f);

    // Apply the filter to get filteredBatchCloud
    pcl::PointCloud<pcl::PointXYZ> filteredBatchCloud;
    voxelFilter.filter(filteredBatchCloud);

    // Create a new point cloud to hold the colored points
    pcl::PointCloud<pcl::PointXYZRGB> coloredBatchCloud;

    // Loop through each point in the filtered batch
    for (const auto& point : filteredBatchCloud.points)
    {
      pcl::PointXYZRGB coloredPoint;

      // Copy XYZ coordinates
      coloredPoint.x = point.x;
      coloredPoint.y = point.y;
      coloredPoint.z = point.z;

      // Calculate distance to currentPose
      float distance = std::sqrt(std::pow(point.x - currentPose[0], 2) +
                                 std::pow(point.y - currentPose[1], 2) +
                                 std::pow(point.z - currentPose[2], 2));

      // Set color based on proximity, multiplier is set assuming max dist is 30mm and min is 0 mm
      uint8_t r = static_cast<uint8_t>(255 * std::exp(-distance*0.1));
      uint8_t g = 0;
      uint8_t b = static_cast<uint8_t>(255 * (1 - std::exp(-distance*0.1)));

      // ROS_INFO("r: %u, g: %u, b: %u", static_cast<unsigned int>(r), static_cast<unsigned int>(g), static_cast<unsigned int>(b));

      // Combine RGB into a single uint32_t
      uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
      coloredPoint.rgb = *reinterpret_cast<float*>(&rgb);

      // Add the colored point to the colored batch
      coloredBatchCloud.points.push_back(coloredPoint);
    }

    // Append the colored points to filteredCloud
    filteredCloud += coloredBatchCloud;

    // Update lastFilter
    lastFilter += batchSize;
  }
}

// pcl::PolygonMesh PointCloudProcessor::createMesh()
// {
//     // Create kd-tree for point cloud
//     pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);

//     // Estimate normals
//     pcl::PointCloud<pcl::PointNormal>::Ptr normals(new pcl::PointCloud<pcl::PointNormal>);
//     pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointNormal> normalEstimation;
//     normalEstimation.setInputCloud(filteredCloud);
//     normalEstimation.setSearchMethod(tree);
//     normalEstimation.setRadiusSearch(0.05);
//     normalEstimation.compute(*normals);

//     // Greedy Projection Triangulation
//     pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
//     pcl::PolygonMesh triangles;

//     gp3.setSearchRadius(0.025);
//     gp3.setMu(2.5);
//     gp3.setMaximumNearestNeighbors(100);
//     gp3.setMaximumSurfaceAngle(M_PI / 4);
//     gp3.setMinimumAngle(M_PI / 18);
//     gp3.setMaximumAngle(2 * M_PI / 3);
//     gp3.setNormalConsistency(false);

//     gp3.setInputCloud(normals);
//     gp3.setSearchMethod(tree);
//     gp3.reconstruct(triangles);

//     // Laplace Smoothing
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr smoothedCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//     pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointNormal> mls;

//     mls.setInputCloud(filteredCloud);
//     mls.setSearchMethod(tree);
//     mls.setSearchRadius(0.03);
//     mls.setPolynomialFit(true);

//     pcl::PointCloud<pcl::PointNormal> mlsPoints;
//     mls.process(mlsPoints);

//     // Convert mlsPoints back to a pcl::PointCloud<pcl::PointXYZRGB>
//     pcl::PointCloud<pcl::PointXYZRGB> mlsXYZRGB;
//     for (const auto& point : mlsPoints)
//     {
//         pcl::PointXYZRGB newPoint;
//         newPoint.x = point.x;
//         newPoint.y = point.y;
//         newPoint.z = point.z;
//         // Optionally, you can also copy the RGB values if needed.
//         mlsXYZRGB.push_back(newPoint);
//     }

//     // Replace the vertices in the mesh with the smoothed vertices
//     pcl::toPCLPointCloud2(mlsXYZRGB, triangles.cloud);

//     return triangles;
// }

sensor_msgs::PointCloud2 PointCloudProcessor::createPointCloud(std::vector<Eigen::Vector2d> contours, geometry_msgs::PoseWithCovarianceStamped pose)
{

  Eigen::Vector3d currentPose(pose.pose.pose.position.x, pose.pose.pose.position.y, pose.pose.pose.position.z);
  double norm = (currentPose - previousPose).norm();

  std::vector<Eigen::Vector3d> projectedContours_;

  // only use these points if the pose has moved more than 1mm
  if (norm > 0.8)
  {
    // Project the incoming contours to the global frame
    projectedContours_ = projectContours(contours, pose);

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

    previousPose = currentPose;
  }

  applyVoxelFilterAndColour(currentPose);

  // Convert to sensor_msgs::PointCloud2
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(filteredCloud, output); // Note: Now 'filteredCloud' holds the filtered points

  return output;
}
