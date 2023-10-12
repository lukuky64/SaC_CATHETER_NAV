#include "main.h"
#include <iostream>


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

int main(int argc, char **argv)
{
    // Print to the terminal
    std::cout << "Hello, World!" << std::endl;

    return 0;

    // User input -> Would you like to visualise the system? (yes/no)
    // once user input is 'yes', roslaunch RVIZ, delay for number of seconds to allow it to load

    // run sensor publishing node, image processing node, marker publishing node and centre prediction node
    // The publshing node should wait 5 seconds, publish the initial location but pause before publishing the rest (for the sake of the fake data)

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
