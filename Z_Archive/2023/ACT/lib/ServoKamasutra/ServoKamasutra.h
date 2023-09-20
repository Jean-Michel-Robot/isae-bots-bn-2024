/*
  Allows for the creation of servo objects with a given list of accessible positions, hence the joke about Kamasutra
*/

#ifndef SERVOKAMASUTRA_H
#define SERVOKAMASUTRA_H

#include <Arduino.h>
#include <Servo.h>

class ServoKamasutra : private Servo {
  private:
    String id; // id of the motor (not very useful in most cases)
    int pin; // pin where the servo is connected
    int nbPos; // numbers of allowed positions
    int *positions; // list of allowed positions in degres

    int currentPos; // current position of the servo

    void setAngle(int deg); // sets the angle of the servo

  public:
    ServoKamasutra(String id, int pin, int nbPos, int positions[]); // constructor of ServoKamasutra class

    ~ServoKamasutra(); //destructor of ServoKamasutra class

    int getCurrentPos(); // getter for currentPos

    void setPosition(int positionId); // sets the position of the servo among the listed positions

    void setup(); // initializes the servo
};

#endif
