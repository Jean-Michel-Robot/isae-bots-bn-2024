#ifndef __H_MOTORS
#define __H_MOTORS



class GenericMotor
{
public :
    GenericMotor(bool isMotorLeft);
    virtual void commandMotor_p(int value) = 0;
    virtual void commandMotor_v(int value) = 0;

    bool m_is_motor_left;
    static constexpr int MIN_COMMAND_MOTOR = 0; //4
    static constexpr int MAX_COMMAND_MOTOR = 255; //32 //40
};

class OdriveMotorBR : public GenericMotor
{
public :

    OdriveMotorBR(bool isMotorLeft);
    void commandMotor_p(int value)override;
    void commandMotor_v(int value)override;

    static constexpr int MOTOR_LEFT_INDEX = 1;
    static constexpr int MOTOR_RIGHT_INDEX = 0;

private :
    static constexpr int PIN_PWM_R = 22;
    static constexpr int PIN_DIR_R = 23;
    static constexpr int PIN_PWM_L = 9;
    static constexpr int PIN_DIR_L = 10;

    CytronMotor m_motor;

    int m_oldMotorCommand = 0;
};

#endif
