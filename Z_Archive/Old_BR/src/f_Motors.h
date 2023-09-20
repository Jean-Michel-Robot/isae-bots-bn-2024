#ifndef __H_MOTORS
#define __H_MOTORS
#include "a_define.h"
#include "v_Logger.h"

#include "src/CytronMotor/CytronMotor.h"
#include "src/OdriveArduino/ODriveArduino.h"

class GenericMotor
{
public :
    enum DirectionMoteur
    {
      FORWARD = 0,
      REVERSE = 1
    };
    enum Side
    {
        LEFT = 0,
        RIGHT = 1
    };

    GenericMotor(Side m_side, DirectionMoteur m_direction);
    virtual void commandMotor(int value) =0;
    bool isLeft(){ return m_side == LEFT; }

    static constexpr int MIN_COMMAND_MOTOR = 0; //4
    static constexpr int MAX_COMMAND_MOTOR = 255; //32 //40
protected:
    Side m_side;
    DirectionMoteur m_direction;
};

class CytronMotorBR : public GenericMotor
{
public :

    CytronMotorBR(Side m_side, DirectionMoteur m_direction);
    void commandMotor(int value) override;
    static constexpr int MOTOR_LEFT_INDEX = 1;
    static constexpr int MOTOR_RIGHT_INDEX = 0;

private :
    static constexpr int PIN_PWM_R = 9;
    static constexpr int PIN_DIR_R = 10;
    static constexpr int PIN_PWM_L = 22;
    static constexpr int PIN_DIR_L = 23;

    CytronMotor m_motor;

};

#endif
