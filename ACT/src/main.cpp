#include <Arduino.h>
#include "Doors.hpp"
#include "ClampElevator.hpp"
#include "Clamp.hpp"
#include "ClampHoriz.hpp"
#include "a_define.hpp"
#include "Ultrasonic.hpp"
#include <ros.h>

ros::NodeHandle nh;

Doors doors = Doors();
DoorsROS doorsROS = DoorsROS(&doors, &nh);

ClampElevator clampelevator = ClampElevator();
ClampElevatorROS clampelevatorROS = ClampElevatorROS(&clampelevator, &nh);

Clamp clamp = Clamp();
ClampROS clampROS = ClampROS(&clamp, &nh);

ClampHoriz clamphoriz = ClampHoriz(); 
ClampHorizROS clamphorizROS = ClampHorizROS(&clamphoriz, &nh);  

Ultrasonic ultrasonic_l = Ultrasonic(ULTRASONIC_L_TRIG_PIN, ULTRASONIC_L_ECHO_PIN);
Ultrasonic ultrasonic_r = Ultrasonic(ULTRASONIC_R_TRIG_PIN, ULTRASONIC_R_ECHO_PIN);

UltrasonicROS ultrasonicROS = UltrasonicROS(&ultrasonic_l, &ultrasonic_r, &nh);


void setup() {

  nh.initNode();

  doorsROS.setup();
  clampelevatorROS.setup();
  clampROS.setup();
  clamphorizROS.setup();

  ultrasonicROS.setup();


}

void loop() {

  doorsROS.loop();
  clampelevatorROS.loop();
  clampROS.loop();
  clamphorizROS.loop();

  ultrasonicROS.loop();
  

  nh.spinOnce();
}