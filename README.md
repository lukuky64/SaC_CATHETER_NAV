# Sensors and Control Repository

## Getting Started

### Prerequisites
- Ensure you have ROS and Catkin installed on your system.

---

### Clone the Repository

```bash
cd ~/catkin_ws/src
git clone git@github.com:lukuky64/sensors_and_control.git
```

---

### Optional: CPP and ROS Starter
If you want to learn how to interact with ROS, you can also clone the following repository:

```bash
cd ~/catkin_ws/src
git clone https://github.com/lukuky64/CPP_with_ROS.git
```

---

### Build the Files

Navigate to your Catkin workspace and build the project:

```bash
cd ~/catkin_ws
catkin_make
```

---

## Running the Main Simulation

1. Replace the path in the quotes with the path to your data folder:

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

