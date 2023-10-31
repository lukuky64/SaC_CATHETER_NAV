#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "std_msgs/String.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <cstdlib>
#include <mutex>
#include <atomic>
#include <ncurses.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <cmath>

// Global variables
std::atomic<bool> pauseRequested(false);
std::mutex pauseMutex;
double initial_catheter_x = 0.0;
double initial_catheter_y = 0.0;
double initial_catheter_z = 0.0;
double goal_x, goal_y, goal_z;

// Callback function for catheter position
void catheterPositionCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    // Retrieve the position of the catheter from the received message
    double catheter_x = msg->pose.pose.position.x;
    double catheter_y = msg->pose.pose.position.y;
    double catheter_z = msg->pose.pose.position.z;

    // Store the initial position
    if (initial_catheter_x == 0.0 && initial_catheter_y == 0.0 && initial_catheter_z == 0.0)
    {
        initial_catheter_x = catheter_x;
        initial_catheter_y = catheter_y;
        initial_catheter_z = catheter_z;
    }

    // Calculate the Euclidean distance between the current location and the goal location
    double distance = std::sqrt(std::pow((goal_x - catheter_x), 2) + std::pow((goal_y - catheter_y), 2) + std::pow((goal_z - catheter_z), 2));

    // Print or process the calculated distance as needed
    ROS_INFO("Euclidean distance to goal location: %f", distance);
}

// Callback function for camera calibration
void cameraCalibrationCallback(const std_msgs::String::ConstPtr &msg)
{
    // Process the camera calibration data here
    // For example, you can print the received data to the console
    ROS_INFO("Received camera calibration data: %s", msg->data.c_str());
}

void user_input_thread()
{
    // Initialize ncurses
    initscr();
    timeout(100); // Set timeout for getch()

    // User input for system visualization
    std::string visualize;
    printw("Would you like to visualize the system? (yes/no): ");
    refresh();
    std::cin >> visualize;
    endwin(); // End ncurses mode

    if (visualize == "yes")
    {
        system("roslaunch main main.launch");
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }

    // Re-initialize ncurses if needed
    initscr();
    std::string startAlgorithm;  // Declare the variable

    printw("Would you like to start the algorithm? (yes/no): ");
    refresh();
    std::cin >> startAlgorithm;
    endwin(); // End ncurses mode

    if (startAlgorithm == "yes")
    {
        pauseRequested = true;
        system("rosservice call /set_paused 'data: false'");
        printw("Data flow unpaused\n");
    }
    else
    {
        pauseRequested = false;
        system("rosservice call /set_paused 'data: true'");
        printw("Data flow paused\n");
    }

    // User input for end goal location
    printw("Enter the x-coordinate of the end location: ");
    refresh();
    std::cin >> goal_x;

    printw("Enter the y-coordinate of the end location: ");
    refresh();
    std::cin >> goal_y;

    printw("Enter the z-coordinate of the end location: ");
    refresh();
    std::cin >> goal_z;

    if (std::cin.fail())
    {
        printw("Invalid input. Please enter numeric values for the coordinates.\n");
    }

    endwin(); // End ncurses mode
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main_node");
    ros::NodeHandle nh;

    // Create subscribers
    ros::Subscriber sub_calibration = nh.subscribe("camera_calibration_topic", 10, cameraCalibrationCallback);
    ros::Subscriber catheter_sub = nh.subscribe("em_filter_node", 10, catheterPositionCallback);

    std::thread input_thread(user_input_thread);

    ros::AsyncSpinner spinner(1); // Use 1 thread
    spinner.start();

    input_thread.join();

    return 0;
}
