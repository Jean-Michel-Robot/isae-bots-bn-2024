/*
   gere la communication avec la SAbertooth pour controler les moteurs de propulsion
   deux modes existent, un mode série simplifié, et un plus complet (à adapter avec les switchs sur la carte)
*/
#include "a_define.h"
#include "f_Motors.h"
#include "v_Logger.h"
#include "z_Setup_Loop.h"

GenericMotor::GenericMotor(Side side, DirectionMoteur direction): m_side(side), m_direction(direction)
{

}

CytronMotorBR::CytronMotorBR(Side side, DirectionMoteur direction):
    GenericMotor(side,direction),
    m_motor(side == LEFT ? MOTOR_LEFT_INDEX:MOTOR_RIGHT_INDEX,
            side == LEFT ? PIN_PWM_L:PIN_PWM_R,
            side == LEFT ? PIN_DIR_L:PIN_DIR_R,
            MIN_COMMAND_MOTOR, MAX_COMMAND_MOTOR)
{
  m_motor.initMotor();
}

void CytronMotorBR::commandMotor(int value)
{
    if(isLeft()){
      Logger::setFieldValue(value, Logger::commandeMotorL);
    }
    else{
      Logger::setFieldValue(value, Logger::commandeMotorR);
    }
    m_motor.commandMotor(value * (m_direction == FORWARD ? 1 : -1));
}
