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
#include <geometry_msgs/PoseWithCovarianceStamped.h>

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
extern double initial_catheter_x;
extern double initial_catheter_y;
extern double initial_catheter_z;
extern double goal_x;
extern double goal_y;
extern double goal_z;

#endif // MAIN_H

