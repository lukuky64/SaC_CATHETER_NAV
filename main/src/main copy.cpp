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

// Define the global variables
double initial_catheter_x = 0.0;
double initial_catheter_y = 0.0;
double initial_catheter_z = 0.0;
double goal_x, goal_y, goal_z;

// Constructor
Main::Main()
{
    // do something
}

// Destructor
Main::~Main()
{
    // do something
}

// Callback function for the subscriber
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
// Define a callback function for the camera calibration subscriber
void cameraCalibrationCallback(const std_msgs::String::ConstPtr &msg)
{
    // Process the camera calibration data here
    // For example, you can print the received data to the console
    ROS_INFO("Received camera calibration data: %s", msg->data.c_str());
}

std::atomic<bool> pauseRequested(false);
std::mutex pauseMutex;

// Define the interrupt service routine
// you will need to install this sudo apt-get install libncurses5-dev libncursesw5-dev

void interruptServiceRoutine()
{
    bool paused = false;
    while (true)
    {
        int ch = getch(); // Get the character from the terminal
        if (ch != ERR)
        {
            if (ch == 'p' && !paused)
            {
                std::lock_guard<std::mutex> lock(pauseMutex);
                pauseRequested = true;
                paused = true;
                std::cout << "Pausing the code..." << std::endl;
            }
            else if (ch == 'r' && paused)
            {
                std::lock_guard<std::mutex> lock(pauseMutex);
                pauseRequested = false;
                paused = false;
                std::cout << "Resuming the code..." << std::endl;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// Create a thread for the interrupt service routine
std::thread isr(interruptServiceRoutine);

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "main_node");
    ros::NodeHandle nh;

    // Create subscribers
    ros::Subscriber sub_calibration = nh.subscribe("camera_calibration_topic", 10, cameraCalibrationCallback);
    // ros::Subscriber sub_prediction = nh.subscribe("ros_predict_node", 1000, predictionImageCallback);
    ros::Subscriber catheter_sub = nh.subscribe("em_filter_node", 10, catheterPositionCallback);

    // User input for system visualization
    std::string visualize;
    std::cout << "Would you like to visualize the system? (yes/no): ";
    std::cin >> visualize;

    if (visualize == "yes")
    {
        // Launch RVIZ using the system function
        system("roslaunch main main.launch");

        // Add a delay to allow time for RVIZ to load
        std::chrono::seconds delaySeconds(5); // Change the duration as needed
        std::this_thread::sleep_for(delaySeconds);
    }

    // run sensor publishing node, image processing node, marker publishing node

    // Run the sensor publishing node
    // system("rosrun sensor_publishing sensor_publishing_sensor_publisher");

    // Run the image processing node
    // system("rosrun image_processing image_processing_node");

    // Run the marker publishing node
    // system("rosrun marker_publishing marker_publishing_node");

    // The publishing node should wait 5 seconds, publish the initial location but pause before publishing the rest (for the sake of the fake data)
    std::chrono::seconds delaySeconds(5); // Change the duration as needed
    std::this_thread::sleep_for(delaySeconds);

    // run the camera calibration node on the first image and find the scale value
    // system("rosrun camera_calibration camera_calibration_node");
    // center prediction node (input the scale value)

    // User input -> Would you like to start the algorithm? (yes/no)
    std::string startAlgorithm;
    std::cout << "Would you like to start the algorithm? (yes/no): ";
    std::cin >> startAlgorithm;

    if (startAlgorithm == "yes") {
        pauseRequested = true; // Set the initial state to paused
        // Unpause the publishing of the data and let it flow
        int result = system("rosservice call /set_paused 'data: false'");
        
        if (result == 0) {
            ROS_INFO("Data flow unpaused");
        } else {
            ROS_ERROR("Failed to call service /set_paused");
        }
    }
    else
    {
        pauseRequested = false; // Set the initial state to paused
        // Unpause the publishing of the data and let it flow
        int result = system("rosservice call /set_paused 'data: false'");
        
        if (result == 1) {
            ROS_INFO("Data flow paused");
        } else {
            ROS_ERROR("Failed to call service /set_paused");
        }
    }
    // user input for end goal location
    double goal_x, goal_y, goal_z;
    std::cout << "Enter the x-coordinate of the end location: ";
    std::cin >> goal_x;
    std::cout << "Enter the y-coordinate of the end location: ";
    std::cin >> goal_y;
    std::cout << "Enter the z-coordinate of the end location: ";
    std::cin >> goal_z;

    // Check if the input is valid
    if (std::cin.fail())
    {
        std::cerr << "Invalid input. Please enter numeric values for the coordinates." << std::endl;
        return 1; // exit the program with an error code
    }

    isr.join(); // Join the ISR thread before exiting

    // Spin to receive callback messages
    ros::spin();

    return 0;
}
