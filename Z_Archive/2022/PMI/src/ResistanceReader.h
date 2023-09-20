/**
 * @file ResistanceReader.h
 * @brief Implements all the actions required for reading the resistance (analoogRead, servos)
 */

#ifndef RESISTANCE_READER_H
#define RESISTANCE_READER_H

#include "a_parameters.h"
#include "a_utils.h"
#include "src/ServoKamasutra/ServoKamasutra.h"
#include "src/SwitchFiltered/SwitchFiltered.h"
#include "ros.h"
#include "std_msgs/Int16.h"


enum ReaderPosition{
  READER_DEPLOYED = 1,
  READER_RETRACTED = 0
};

/**
 * @brief Class including all actions for reading the resistance
 * 
 */
class ResistanceReader {
    
    private :
    
        int m_readerPin;
                
    public :   

        ServoKamasutra m_leftServo;
        ServoKamasutra m_rightServo; 

        ResistanceReader(int readerPin,
                         int leftServoPin, int *leftPositions,
                         int rightServoPin, int *rightPositions);

        void setup();
        void loop();


        /**
         * @brief Gives the value of the resistance read by the teensy
         * 
         * @return the value of the read resistance (in ohm) 
         */
        int readResistance();

        /**
         * @brief Set the position of a servo
         * 
         * @param side LEFT or RIGHT
         * @param pos  READER_DEPLOYED or READER_RETRACTED
         */
        void setPosition(LeftRight side, ReaderPosition pos);

        /**
         * @brief Checks the state of both bumpers of one side
         * 
         * @param side LEFT or RIGHT
         * @return true iif both dumpers are down
         */
        bool areSwitchesPressed(LeftRight side);  
        
};




#endif
