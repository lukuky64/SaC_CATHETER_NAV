cmake_minimum_required(VERSION 3.0.2)
project(point_cloud_publisher)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(point_cloud_publisher_node src/point_cloud_publisher.cpp)
target_link_libraries(point_cloud_publisher_node ${catkin_LIBRARIES})
