/*
  Allows for the creation of UltrasonicRos objects which manages the ros communication for the 2 ultrasonic sensors
*/

#include "UltrasonicRos.h"

//constructor of UltrasonicRos class
UltrasonicRos::UltrasonicRos(ros::NodeHandle *nh, int nbUltrasonicSensors, UltrasonicSensor **ultrasonicSensorArray, float timePubDistances)
    : timerPubDistances(timePubDistances),
    pubUltrasonicDistances("ultrasonicDistances", &ultrasonicDistanceArray)
{
  this->m_nh = nh;

  this->nbUltrasonicSensors = nbUltrasonicSensors;
  this->ultrasonicSensorArray = ultrasonicSensorArray;
}

void UltrasonicRos::publishUltrasonicDistances()
{
  ultrasonicDistanceArray.x = (int)ultrasonicSensorArray[0]->getDistance();
  ultrasonicDistanceArray.y = (int)ultrasonicSensorArray[1]->getDistance();

  pubUltrasonicDistances.publish(&ultrasonicDistanceArray);
}

void UltrasonicRos::setup()
{
  m_nh->advertise(pubUltrasonicDistances);

  timerPubDistances.reset();
}

void UltrasonicRos::loop()
{
  if (timerPubDistances.startIfNotStartedAndTestExpiration(millis()))
  {
    publishUltrasonicDistances();
    timerPubDistances.reset();
  }
}
