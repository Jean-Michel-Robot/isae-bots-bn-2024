/*
  Allows for the creation of servo objects with a given list of accessible positions, hence the joke about Kamasutra
*/

#include "ServoKamasutra.h"
//#include <assert.h>

// constructor of ServoKamasutra class
ServoKamasutra::ServoKamasutra(String id, int pin, int nbPos, int *positions) : Servo() {
    this->id = id;
    this->pin = pin;
    this->nbPos = nbPos;

    //copy of the positions array so any modification will not affect the object
    this->positions = (int *) malloc(nbPos * sizeof(int));
    //assert(positions != NULL);
    memcpy(this->positions, positions, nbPos * sizeof(int));

    this->currentPos = -1; // not a valid position

}

// destructor of ServoKamasutra class
ServoKamasutra::~ServoKamasutra() {
    free(positions); //frees memory allocated for the positions array inside the object
}

// sets the angle of the servo
void ServoKamasutra::setAngle(int deg) {
    this->write(deg);
}

// getter for currentPos
int ServoKamasutra::getCurrentPos() {
    return currentPos;
}

// sets the position of the servo among the listed positions
void ServoKamasutra::setPosition(int positionId) {
    if (positionId < nbPos) {
        setAngle(positions[positionId]);
        currentPos = positionId; // updates the current position
    }
    else currentPos = -1; // not a valid position

}

int ServoKamasutra::getPosition(int positionId){
    if (positionId < nbPos) {
        return positions[positionId];
    }
    else return -1; // not a valid position
}

void ServoKamasutra::updatePositionAngle(int positionId, int angle){
    if(positionId < nbPos){
        positions[positionId] = angle;
    }
}

void ServoKamasutra::setup() {
    this->attach(pin);
    setPosition(0);
}


