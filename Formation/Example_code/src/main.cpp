/*
  * main.cpp
  * Main file of the code 
*/

// Including libraries

#include <Arduino.h> // Arduino header (does not seem to be  required though...)
#include <ros.h>  // ROS library

#include "Elevator.hpp" // Elevator header file
#include "Clamp.hpp"    // Clamp header file


// ROS node handle
ros::NodeHandle nh;

// Creating elevator instance
Elevator elevator = Elevator();

// Creating ElevatorROS instance
ElevatorROS elevatorROS = ElevatorROS(&elevator, &nh);

// Same for the clamp
Clamp clamp = Clamp();
ClampROS clampROS = ClampROS(&clamp, &nh);

// Setup function of the code
//  it is executed once at the beginning of the program
void setup() {

  // function that start the ROS node handle
  nh.initNode();

  // Setup function of elevatorROS object
  elevatorROS.setup();

}

void loop() {
  
  // function the "refresh" the ROS node handle
  //  it allows to look for messages from the ROS topics
  //  and execute callback functions if needed
  nh.spinOnce();

  // Loop function of elevatorROS object
  elevatorROS.loop();

}
