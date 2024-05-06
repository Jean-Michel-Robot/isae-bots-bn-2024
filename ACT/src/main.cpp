#include <Arduino.h>
#include "Doors.hpp"
#include "ClampElevator.hpp"
#include "Clamp.hpp"
#include "a_define.hpp"
#include <ros.h>

Doors doors;
ClampElevator clampelevator;
Clamp clamp;

ros::NodeHandle nh;

void setup() {

  doors.setup();
  clampelevator.setup();
  clamp.setup();
  
  nh.initNode();
}

void loop() {

  doors.loop();
  clampelevator.loop();
  clamp.loop();

  nh.spinOnce();
}