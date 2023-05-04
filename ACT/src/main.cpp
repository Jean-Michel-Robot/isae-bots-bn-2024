#include <Arduino.h>
#include <ros.h>

#include "Elevator.hpp"
#include "Doors.hpp"

ros::NodeHandle nh;

Doors doors = Doors();
DoorsROS doorsROS = DoorsROS(&doors, &nh);

Elevator elevator = Elevator();
ElevatorROS elevatorROS = ElevatorROS(&elevator, &nh);


void setup() {

  nh.initNode();

  doorsROS.setup();
  elevatorROS.setup();

}

void loop() {
  
  nh.spinOnce();

  doorsROS.loop();
  elevatorROS.loop();

}