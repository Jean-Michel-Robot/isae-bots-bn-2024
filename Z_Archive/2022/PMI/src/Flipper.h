/**
 * @file Flipper.h
 *
 * @brief Implementation of the arm intended to flip the resistances
 * 
 */

#ifndef FLIPPER_H
#define FLIPPER_H


#include "a_parameters.h"
#include "a_utils.h"
#include "src/ServoKamasutra/ServoKamasutra.h"
#include "src/SwitchFiltered/SwitchFiltered.h"

enum FlipperPosition{
  FLIPPER_DEPLOYED = 1,
  FLIPPER_RETRACTED = 0
};


class Flipper {

    private:     

    public:
    
        ServoKamasutra m_leftServo;
        ServoKamasutra m_RightServo;   
        
        Flipper(int leftServoPin, int leftPositions[2],
                int rightServoPin, int rightPositions[2]);

        void setup();
        void loop();

        /**
         * @brief Set the position of a servo
         * 
         * @param side LEFT or RIGHT
         * @param pos  FLIPPER_DEPLOYED or FLIPPER_RETRACTED
         */
        void setPosition(LeftRight servo, FlipperPosition pos);

};

#endif
