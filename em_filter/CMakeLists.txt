cmake_minimum_required(VERSION 3.0.2)
project(em_filter)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES em_filter
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}_node src/filter.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  Eigen3::Eigen  # Eigen3 target
)
