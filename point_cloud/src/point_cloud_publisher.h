#ifndef POINT_CLOUD_PUBLISHER_H
#define POINT_CLOUD_PUBLISHER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class PointCloudPublisher
{
public:
  PointCloudPublisher();
  void spin();

private:
  void publishPointCloud();
  ros::NodeHandle nh_;
  ros::Publisher pcl_pub_;
};

#endif // POINT_CLOUD_PUBLISHER_H