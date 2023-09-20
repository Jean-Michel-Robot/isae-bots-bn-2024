/**
 * @file Flipper.cpp
 * @brief Class for flipping the squares
 */

#include "Flipper.h"

Flipper::Flipper(int leftPin, int leftPositions[2],
                 int rightPin, int rightPositions[2]):
                m_leftServo("Flipper_left", leftPin, 2, leftPositions),
                m_RightServo("Flipper_right",rightPin, 2, rightPositions){
  ;
}

void Flipper::setup(){
  m_leftServo.setup();
  m_RightServo.setup();
}

void Flipper::loop(){
}


void Flipper::setPosition(LeftRight servo, FlipperPosition pos){
  if(servo == LEFT) m_leftServo.setPosition((int)pos);
  else              m_RightServo.setPosition((int)pos);
}
