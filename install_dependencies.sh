#!/bin/bash

# Update package list
sudo apt-get update

# Install ROS packages
sudo apt-get install -y ros-noetic-pcl-conversions
sudo apt-get install -y ros-noetic-pcl-ros
sudo apt-get install -y ros-noetic-cv-bridge
sudo apt-get install -y ros-noetic-roscpp
sudo apt-get install -y ros-noetic-sensor-msgs
sudo apt-get install -y ros-noetic-rospy
sudo apt-get install -y ros-noetic-std-msgs
sudo apt-get install -y ros-noetic-geometry-msgs
sudo apt-get install -y ros-noetic-visualization-msgs
sudo apt-get install -y ros-noetic-tf

# Install C++ libraries
sudo apt-get install -y libopencv-dev

# Install Python3
sudo apt-get install -y python3

# Install Python packages
pip3 install tensorflow

