#include <Arduino.h>
#include <ros.h>

#include "Elevator.hpp"
#include "Doors.hpp"
#include "Clamp.hpp"
#include "Cherry.hpp"

ros::NodeHandle nh;

Doors doors = Doors();
DoorsROS doorsROS = DoorsROS(&doors, &nh);

Elevator elevator = Elevator();
ElevatorROS elevatorROS = ElevatorROS(&elevator, &nh);

Cherry cherry = Cherry();
CherryROS cherryROS = CherryROS(&cherry, &nh);

Clamp clamp = Clamp();
ClampROS clampROS = ClampROS(&clamp, &nh);

void setup() {

  nh.initNode();

  doorsROS.setup();
  elevatorROS.setup();
  cherryROS.setup();
  clampROS.setup();

}

void loop() {
  
  nh.spinOnce();

  doorsROS.loop();
  elevatorROS.loop();
  cherryROS.loop();
  clampROS.loop();

}