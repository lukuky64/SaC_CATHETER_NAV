#include "point_cloud_publisher.h"
#include <pcl_conversions/pcl_conversions.h>

PointCloudPublisher::PointCloudPublisher()
{
  pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/point_cloud_topic", 10);
}

void PointCloudPublisher::spin()
{
  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    publishPointCloud();
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void PointCloudPublisher::publishPointCloud()
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  // Fill in the cloud data (e.g., read from a file or generate programmatically)
  cloud.width = 5;
  cloud.height = 1;
  cloud.points.resize(cloud.width * cloud.height);

  for (size_t i = 0; i < cloud.points.size(); ++i)
  {
    cloud.points[i].x = static_cast<float>(i);
    cloud.points[i].y = static_cast<float>(i);
    cloud.points[i].z = static_cast<float>(i);
  }

  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(cloud, output);
  output.header.frame_id = "base_link";
  pcl_pub_.publish(output);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "point_cloud_publisher");
  PointCloudPublisher publisher;
  publisher.spin();
  return 0;
}