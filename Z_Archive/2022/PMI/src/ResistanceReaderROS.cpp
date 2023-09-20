/**
 * @file ResistanceReaderROS.cpp
 * 
 */
#include "ResistanceReaderROS.h"

ResistanceReader* ResistanceReaderROS::m_p_resistanceReader = NULL;

ResistanceState ResistanceReaderROS::m_state = RESISTANCE_IDLE;
unsigned long ResistanceReaderROS::m_nextMeasureMillis = 0;
unsigned long ResistanceReaderROS::m_timeout = 0;
LeftRight ResistanceReaderROS::m_currentSide = LEFT;


ResistanceReaderROS::ResistanceReaderROS(ResistanceReader* p_resistanceReader, ros::NodeHandle* p_nh) :                                   
                                         m_pubResistanceValue("res_feedback", &m_resistanceValueMsg),
                                         m_subResistanceCall("res_request", launchReadingCb),
                                         m_subResistanceUpdateAngles("resistance_update", updateAnglesCb){

  m_p_nh = p_nh;
  m_p_resistanceReader = p_resistanceReader;
  for(int i=0; i<4; i++)  m_measureArray[i] = 0;
  
}

void ResistanceReaderROS::setup(){
  
  m_p_resistanceReader->setup();

  m_p_nh->subscribe(m_subResistanceCall);
  m_p_nh->subscribe(m_subResistanceUpdateAngles);
  m_p_nh->advertise(m_pubResistanceValue);
  m_state = RESISTANCE_IDLE;
  m_currentSide = RIGHT;

}

void ResistanceReaderROS::loop(){

  if(m_state == RESISTANCE_DEPLOYING){
    if(m_p_resistanceReader->areSwitchesPressed(m_currentSide)){ 
      m_state = RESISTANCE_MEASURING;
      m_nextMeasureMillis = millis() + RESISTANCE_MEASURE_DELAY;
    }

    /* Timeout, we retract the arm and send an error message */
    else if(millis() > m_timeout){
      m_resistanceValueMsg.data = 3;
      m_pubResistanceValue.publish(&m_resistanceValueMsg);
      m_p_resistanceReader->setPosition(m_currentSide, READER_RETRACTED);
      m_state = RESISTANCE_IDLE;
      m_currentMeasureNumber = 0;
      for(int i=0; i<4; i++)  m_measureArray[i] = 0;
    }
  }
  
  else if(m_state == RESISTANCE_MEASURING){

      if(m_currentMeasureNumber < RESISTANCE_MEASURE_NUMBER && millis() > m_nextMeasureMillis){

        int resistanceValue = m_p_resistanceReader->readResistance();
        
        if(isBetween(resistanceValue, RESISTANCE_YELLOW_MIN, RESISTANCE_YELLOW_MAX))  m_measureArray[0]++;
        else if(isBetween(resistanceValue, RESISTANCE_PURPLE_MIN, RESISTANCE_PURPLE_MAX))  m_measureArray[1]++;
        else if(isBetween(resistanceValue, RESISTANCE_CROSS_MIN, RESISTANCE_CROSS_MAX))    m_measureArray[2]++;
        else m_measureArray[3]++;

        m_currentMeasureNumber++;
        m_nextMeasureMillis = millis() + RESISTANCE_MEASURE_DELAY;
      }

      /* All measures have been done, we retract the arm and send the found value of resistance */
      else if(m_currentMeasureNumber >= RESISTANCE_MEASURE_NUMBER){ 

        if(m_measureArray[0] > RESISTANCE_MEASURE_NB_MIN) m_resistanceValueMsg.data = 0;
        else if(m_measureArray[1] > RESISTANCE_MEASURE_NB_MIN) m_resistanceValueMsg.data = 1;
        else if(m_measureArray[2] > RESISTANCE_MEASURE_NB_MIN) m_resistanceValueMsg.data = 2;
        else m_resistanceValueMsg.data = 3;

        m_pubResistanceValue.publish(&m_resistanceValueMsg);
        m_p_resistanceReader->setPosition(m_currentSide, READER_RETRACTED);
        m_state = RESISTANCE_IDLE;
        m_currentMeasureNumber = 0;
        for(int i=0; i<4; i++)  m_measureArray[i] = 0;
      }
  }
}

void ResistanceReaderROS::launchReadingCb(const std_msgs::Int16 &rosLaunchMsg){
    if(m_state == RESISTANCE_IDLE){
      if(rosLaunchMsg.data == 0)  m_currentSide = RIGHT;
      else m_currentSide = LEFT;
      m_p_resistanceReader->setPosition(m_currentSide, READER_DEPLOYED);
      m_state = RESISTANCE_DEPLOYING;
      m_timeout = millis() + RESISTANCE_TIMEOUT;
    }

}

void ResistanceReaderROS::updateAnglesCb(const std_msgs::Int16MultiArray &arrayMsg){

  m_p_resistanceReader->m_leftServo.updatePositionAngle(READER_RETRACTED, arrayMsg.data[0]);
  m_p_resistanceReader->m_leftServo.updatePositionAngle(READER_DEPLOYED, arrayMsg.data[1]);
  m_p_resistanceReader->m_leftServo.updatePositionAngle(READER_RETRACTED, arrayMsg.data[2]);
  m_p_resistanceReader->m_leftServo.updatePositionAngle(READER_DEPLOYED, arrayMsg.data[3]);
  
}


