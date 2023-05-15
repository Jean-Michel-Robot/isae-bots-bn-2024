#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Int16.h>
#include <Servo.h>

#include "Elevator.hpp"
#include "Doors.hpp"
#include "Clamp.hpp"
#include "Cherry.hpp"
#include "Ultrasonic.hpp"
#include "Deguisement.hpp"

ros::NodeHandle nh;

Doors doors = Doors();
DoorsROS doorsROS = DoorsROS(&doors, &nh);

Elevator elevator = Elevator();
ElevatorROS elevatorROS = ElevatorROS(&elevator, &nh);

Cherry cherry = Cherry();
CherryROS cherryROS = CherryROS(&cherry, &nh);

Clamp clamp = Clamp();
ClampROS clampROS = ClampROS(&clamp, &nh);

Ultrasonic ultrasonic_l = Ultrasonic(ULTRASONIC_L_TRIG_PIN, ULTRASONIC_L_ECHO_PIN);
Ultrasonic ultrasonic_r = Ultrasonic(ULTRASONIC_R_TRIG_PIN, ULTRASONIC_R_ECHO_PIN);

UltrasonicROS ultrasonicROS = UltrasonicROS(&ultrasonic_l, &ultrasonic_r, &nh);

// DeguisementROS deguisementROS = DeguisementROS(&nh);


void setup() {

  nh.initNode();

  doorsROS.setup();
  elevatorROS.setup();
  cherryROS.setup();
  clampROS.setup();

  ultrasonicROS.setup();

  // deguisementROS.setup();

}

void loop() {
  
  nh.spinOnce();

  doorsROS.loop();
  elevatorROS.loop();
  cherryROS.loop();
  clampROS.loop();

  ultrasonicROS.loop();

  // deguisementROS.loop();

}
