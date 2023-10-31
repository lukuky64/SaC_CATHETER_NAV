# Sensors and Control Repository

## Getting Started

### Prerequisites

For easy installation from Ubuntu terminal, run [this](install_dependencies.sh) like so: `./install_dependencies.sh` 

For individual installation:
 
 <details>
  
- Ubuntu 20.04 (https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview)
- ROS
```Bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
sudo apt-get install ros-noetic-desktop-full
```
- Catkin
```Bash
sudo apt-get install ros-noetic-catkin
```
- pcl-conversions
```Bash
sudo apt-get install ros-noetic-pcl-conversions
```
- Python3
```Bash
sudo apt-get update
sudo apt-get install python3
```
- Tensorflow
```Bash
pip3 install tensorflow
```
- libopencv-dev
```Bash
sudo apt-get install libopencv-dev
```
---
</details>

### Clone the Repository in your catkin workspace and build:
<details>
 
```bash
cd ~/catkin_ws/src
git clone git@github.com:lukuky64/SaC_CATHETER_NAV.git
```

Navigate to your Catkin workspace and build the project:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

</details>

## Running the Main Simulation
<details>
 
1. Set the environment variable (replace the path in the quotes with the path to your `data folder`):
 
```bash
export MY_PATH="/home/lukuky64/Desktop/Aortic_catheter_project"
```

2. Launch the main simulation:
```bash
roslaunch main main.launch
```
---

3. Unpause publisher:
```Bash
rosservice call /set_paused "data: false" 
```

To pause again:
```Bash
rosservice call /set_paused "data: true" 
```
</details>
