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

DeguisementROS deguisementROS = DeguisementROS(&nh);


void setup() {

  nh.initNode();

  doorsROS.setup();
  elevatorROS.setup();
  cherryROS.setup();
  clampROS.setup();

  ultrasonicROS.setup();

  deguisementROS.setup();

  // pinMode(ULTRASONIC_L_ECHO_PIN, INPUT);
  // pinMode(ULTRASONIC_L_TRIG_PIN, OUTPUT);

}

void loop() {
  
  nh.spinOnce();

  doorsROS.loop();
  elevatorROS.loop();
  cherryROS.loop();
  clampROS.loop();

  digitalWrite(ULTRASONIC_L_TRIG_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(ULTRASONIC_L_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_L_TRIG_PIN, LOW);


  ultrasonicROS.loop();

  // float duration = pulseIn(ULTRASONIC_L_ECHO_PIN, HIGH);

  // ultrasonicROS.m_distance_msg.x = duration / 29.1;
  // ultrasonicROS.m_pub.publish(&ultrasonicROS.m_distance_msg);

  deguisementROS.loop();
  

}
