
# Sensor Publishing with ROS and OpenCV

This README provides instructions for setting up and running a ROS node to publish sensor data. The sensor data is processed using OpenCV.

## Table of Contents

- [Dependencies](#dependencies)
- [Installation](#installation)
- [Build the Package](#build-the-package)
- [Data Directory](#data-directory)
- [Run the Node](#run-the-node)
- [Viewing the Data](#viewing-the-data)

---

## Dependencies

### Install OpenCV

#### Quick Install:

```bash
sudo apt update
sudo apt install libopencv-dev python3-opencv
```

#### Verify Installation:

```bash
python3 -c "import cv2; print(cv2.__version__)"
```

---

## Installation (if you haven't done so already)

Clone the repository and navigate to the project directory.

```bash
cd ~/catkin_ws/src
git clone git@github.com:lukuky64/sensors_and_control.git
cd <sensors_and_control>
```

---

## Build the Package

To build a specific package, use the following command:

```bash
catkin_make --pkg sensor_publishing
```

---

## Data Directory

Ensure that the base data folder is named `Aortic_catheter_project`.

Change directory to the data folder location (eg):

```bash
cd ~/Desktop/Aortic_catheter_project
```

---

## Run the Node

To run the node, execute the following command:

```bash
rosrun sensor_publishing sensor_publishing_ultrasound_pub
```

---

## Viewing the Data

To view the published data, you'll need to open another terminal and run:

```bash
rqt_image_view
```

Then, select the appropriate image node from the dropdown menu.

---
