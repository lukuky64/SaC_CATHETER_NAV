#ifndef MAIN_H
#define MAIN_H

#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "std_msgs/String.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <cstdlib>
#include <mutex>
#include <atomic>

class Main
{
public:
    Main();
    ~Main();

    // Define a callback function for the camera calibration subscriber
    void cameraCalibrationCallback(const std_msgs::String::ConstPtr &msg);

    // Define a callback function for the prediction image subscriber
    void predictionImageCallback(const YourMessageType::ConstPtr &msg);

private:
    std::atomic<bool> pauseRequested;
    std::mutex pauseMutex;
    std::thread isr;
    // Add your private variables here

};

#endif // MAIN_H
