#ifndef MAIN_H
#define MAIN_H

#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "std_msgs/String.h"
#include <iostream>
#include <mutex>
#include <atomic>
#include <thread>
#include <ncurses.h>

class Main
{
public:
    Main();  // Constructor
    ~Main(); // Destructor
};

// Function prototypes
void cameraCalibrationCallback(const std_msgs::String::ConstPtr &msg);
void predictionImageCallback(const std_msgs::String::ConstPtr &msg);
void catheterPositionCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
void interruptServiceRoutine();

extern std::atomic<bool> pauseRequested;
extern std::mutex pauseMutex;
extern std::thread isr;

#endif // MAIN_H
