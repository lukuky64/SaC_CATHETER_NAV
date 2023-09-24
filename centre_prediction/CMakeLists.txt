cmake_minimum_required(VERSION 3.0.2)
project(centre_prediction)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS cv_bridge rospy sensor_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Mark Python scripts for installation
catkin_install_python(PROGRAMS
  src/ros_predict_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
