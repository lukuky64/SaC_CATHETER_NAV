cmake_minimum_required(VERSION 3.0.2)
project(marker_publishing)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  visualization_msgs
  tf
)

find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}_node src/marker_publisher.cpp)

target_link_libraries(marker_publishing_node
  ${catkin_LIBRARIES}
)
