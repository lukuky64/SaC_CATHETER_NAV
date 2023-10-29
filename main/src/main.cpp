#include "main.h"
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

// Define a callback function for the camera calibration subscriber
void cameraCalibrationCallback(const std_msgs::String::ConstPtr &msg)
{
    // Process the camera calibration data here
    // For example, you can print the received data to the console
    ROS_INFO("Received camera calibration data: %s", msg->data.c_str());
}

// Define a callback function for the prediction_image subscriber
void predictionImageCallback(const std_msgs::String::ConstPtr &msg)
{
    // Process the prediction image data here
    // Implement the LOOP logic here
    // Subscribe to the location of the catheter (EM data at the moment) and check if it has reached its end goal location
    // Publish estimated time and distance until the goal location
    // Exit the loop when the end goal is reached (within some tolerance)
}

std::atomic<bool> pauseRequested(false);
std::mutex pauseMutex;

// Define the interrupt service routine
//you will need to install this sudo apt-get install libncurses5-dev libncursesw5-dev
void interruptServiceRoutine()
{
    while (true)
    {
        int ch = getch(); // Get the character from the terminal
        if (ch != ERR)
        {
            if (ch == 'p')
            {
                std::lock_guard<std::mutex> lock(pauseMutex);
                pauseRequested = true;
                std::cout << "Pausing the code..." << std::endl;
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

    // Create a node handle
    ros::NodeHandle nh;

    // Create a subscriber for the camera calibration node
    ros::Subscriber sub_calibration = nh.subscribe("camera_calibration_topic", 10, cameraCalibrationCallback);

    // Create a subscriber for the prediction_image topic
    ros::Subscriber sub_prediction = nh.subscribe("ros_predict_node", 1000, predictionImageCallback);

    // User input -> Would you like to visualise the system? (yes/no)
    std::string visualize;
    std::cout << "Would you like to visualise the system? (yes/no): ";
    std::cin >> visualize;

    if (visualize == "yes")
    {
        // Launch RVIZ using the system function
        system("roslaunch rviz rviz");

        // Add a delay to allow time for RVIZ to load
        std::chrono::seconds delaySeconds(5); // Change the duration as needed
        std::this_thread::sleep_for(delaySeconds);
    }

    // run sensor publishing node, image processing node, marker publishing node

    // Run the sensor publishing node
    system("rosrun sensor_publishing sensor_publishing_node");

    // Run the image processing node
    system("rosrun image_processing image_processing_node");

    // Run the marker publishing node
    system("rosrun marker_publishing marker_publishing_node");


    // The publishing node should wait 5 seconds, publish the initial location but pause before publishing the rest (for the sake of the fake data)
    std::chrono::seconds delaySeconds(5); // Change the duration as needed
    std::this_thread::sleep_for(delaySeconds);

    // run the camera calibration node on the first image and find the scale value
    system("rosrun camera_calibration camera_calibration_node");
    // center prediction node (input the scale value)

    // User input -> Would you like to start the algorithm? (yes/no)
    std::string startAlgorithm;
    std::cout << "Would you like to start the algorithm? (yes/no): ";
    std::cin >> startAlgorithm;

    
    if (startAlgorithm == "yes") {
    // Unpause the publishing of the data and let it flow
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>("rosservice call /set_paused");
    std_srvs::Empty srv;
    srv.request.data = false;  // Setting the data field to false

    if (client.call(srv))
    {
        ROS_INFO("Data flow unpaused");
    }
    else
    {
        ROS_ERROR("Failed to call service /set_paused");
    }
}

    

    isr.join(); // Join the ISR thread before exiting

    return 0;



}

