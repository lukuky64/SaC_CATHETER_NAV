## To Run:
```Bash
roscore
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 world map
rosrun sensor_publishing sensor_publishing_sensor_publisher
rosrun marker_publishing marker_publishing_node
rviz
```
In rviz, add a marker array and select the topic
