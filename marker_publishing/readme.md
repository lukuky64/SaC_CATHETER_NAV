# Publishing EM data as markers

## Prerequisites
Ensure the `sensor_publishing_sensor_publisher` node is operational before proceeding.

---

## Build
```Bash
~/catkin_ws
catkin_make
```
---

## Running the Project

### To Run
Follow these steps, executing each command in a separate terminal window:

1. **Start the ROS master:**
    ```bash
    roscore
    ```
2. **Run the static transform publisher:**
    ```bash
    rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 world map
    ```
3. **Run the sensor publishing node:**
    ```bash
    rosrun sensor_publishing sensor_publishing_sensor_publisher
    ```
4. **Run the marker publishing node:**
    ```bash
    rosrun marker_publishing marker_publishing_node
    ```
5. **Open RViz:**
    ```bash
    rviz
    ```

#### RViz Configuration
In RViz:
- Add a marker array.
- Select the appropriate topic.

---

## License
Include license information here.

---
