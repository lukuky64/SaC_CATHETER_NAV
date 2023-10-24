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

std::atomic<bool> pauseRequested(false);
std::mutex pauseMutex;


//can change this to a gui input
void interruptServiceRoutine()
{
    std::string userInput;
    std::cout << "Press 'p' to pause: ";
    std::cin >> userInput;
    if (userInput == "p")
    {
        std::lock_guard<std::mutex> lock(pauseMutex);
        pauseRequested = true;
        std::cout << "Pausing the code..." << std::endl;
    }
}
std::thread isr(interruptServiceRoutine);
int main(int argc, char **argv)
{
    

    // Initialize the ROS node
    ros::init(argc, argv, "main_node");

    // Create a node handle
    ros::NodeHandle nh;

    // Create a subscriber for the camera calibration node
    ros::Subscriber sub = nh.subscribe("camera_calibration_topic", 1000, cameraCalibrationCallback);


    // User input -> Would you like to visualise the system? (yes/no)
    std::string visualize;
    std::cout << "Would you like to visualise the system? (yes/no): ";
    std::cin >> visualize;

    if (visualize == "yes") {
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
        // Unpause the publishing of the data and let it flow (rosservice call /set_paused "data: false") but do it through C++ not terminal
        ros::NodeHandle nh;
        ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>("/set_paused");
        std_srvs::Empty srv;
        if (client.call(srv))
        {
            ROS_INFO("Data flow unpaused");
        }
        else
        {
            ROS_ERROR("Failed to call service /set_paused");
        }
    }

    // LOOP
    // Subscribe to the location of the catheter (EM data at the moment) and check if it has reached its end goal location
    // Publish estimated time and distance until the goal location
    // LOOP

    // Exit the loop when the end goal is reached (within some tolerance)

    // Consider:
    // Create some interrupt service routine that will pause the data publishing (and moving the robot) when the user presses the space bar or something
    // This would be cooler if we could make some sort of GUI to do this if we have time

    isr.join(); // Join the ISR thread before exiting

    return 0;

    // User input -> Would you like to visualise the system? (yes/no)
    // once user input is 'yes', roslaunch RVIZ, delay for number of seconds to allow it to load

    // run sensor publishing node, image processing node, marker publishing node
    // The publshing node should wait 5 seconds, publish the initial location but pause before publishing the rest (for the sake of the fake data)
    // run the camera calibration node on the first image and find scale value
    // centre prediction node(input the scale value)

    // User input -> Would you like to start the algorithm? (yes/no)
    // once user input is 'yes'
    // unpause the publishing of the data and let it flow (rosservice call /set_paused "data: false") but do it through c++ not terminal

    // LOOP
    // subscribe to the location of the catheter (EM data at the moment) and check if it has reached its end goal location
    // publish estimated time and distance until from goal location
    // LOOP

    // Exit loop when end goal is reached (within some tolerance)




    // Consider:
    // create some interupt service routine that will pause the data publishing (and moving the robot) when the user presses space bar or something 
    // this would be cooler if we could make some sort of GUI to do this if we have time

}

