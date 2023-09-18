cmake_minimum_required(VERSION 3.0.2)
project(marker_publishing)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  visualization_msgs
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/marker_publisher.cpp)

target_link_libraries(marker_publishing_node
  ${catkin_LIBRARIES}
)
