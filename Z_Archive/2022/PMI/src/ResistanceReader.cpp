/**
 * @file ResistanceReader.cpp
 * @brief Implements all the actions required for reading the resistance (analoogRead, servos)
 */
 
#include "ResistanceReader.h"

ResistanceReader::ResistanceReader(int readerPin,
                                   int leftServoPin, int leftPositions[2],
                                   int rightServoPin, int rightPositions[2]) :
                                   m_leftServo("Left_Resistance",leftServoPin,2,leftPositions),
                                   m_rightServo("Right_Resistance",rightServoPin,2,rightPositions){
    this->m_readerPin = readerPin;
}

void ResistanceReader::setup(){
    pinMode(this->m_readerPin, INPUT);
    m_leftServo.setup();
    m_rightServo.setup();
}

void ResistanceReader::loop(){
}

int ResistanceReader::readResistance(){
  int read = analogRead(this->m_readerPin);
  if(read == 0) return 100000;
  else          return RESISTANCE_READ_R0 * (1024./analogRead(this->m_readerPin) - 1);
}

void ResistanceReader::setPosition(LeftRight servo, ReaderPosition pos){
  if(servo == LEFT) m_leftServo.setPosition((int)pos);
  else              m_rightServo.setPosition((int)pos);
}

bool ResistanceReader::areSwitchesPressed(LeftRight bumper){
  return (this->readResistance() < 10000);
}
