/*
  Allows for the creation of UltrasonicRos objects which manages the ros communication for the 2 ultrasonic sensors
*/

#ifndef ULTRASONICROS_H
#define ULTRASONICROS_H

#include <Arduino.h>
#include "UltrasonicSensor.h"

#include <ros.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int16.h>


#include "src/Timer/Timer.h"

class UltrasonicRos
{
private:
  ros::NodeHandle *m_nh;

  Timer timerPubDistances;

  int nbUltrasonicSensors;
  UltrasonicSensor **ultrasonicSensorArray;

public:
  geometry_msgs::Point ultrasonicDistanceArray;
  ros::Publisher pubUltrasonicDistances;

  UltrasonicRos(ros::NodeHandle *nh, int nbUltrasonicSensors, UltrasonicSensor **ultrasonicSensorArray, float timePubDistances); //constructor of UltrasonicRos class

  void publishUltrasonicDistances();

  void setup();

  void loop();
};

#endif