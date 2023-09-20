# GET STARTED
--

## Clone this REPO:
```Ruby
cd ~/catkin_ws/src
git clone git@github.com:lukuky64/sensors_and_control.git
```

## CPP and ROS starter (if you want to learn how to interact with ros):
```ruby
cd ~/catkin_ws/src
git clone https://github.com/lukuky64/CPP_with_ROS
```

## Build the files:
```ruby
cd ~/catkin_ws
catkin_make
```

## Running the main simulation:
Replace the path in `""` with path to the data folder
```ruby
export MY_PATH="/home/lukuky64/Desktop/Aortic_catheter_project"
roslaunch main main.launch
````
