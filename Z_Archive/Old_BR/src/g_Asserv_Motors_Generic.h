#ifndef __H_ASSERV_MOTORS_FAKE
#define __H_ASSERV_MOTORS_FAKE
#include "src/Task/Task.h"
#include "f_Motors.h"
#include "src/asservPID/asservPID.h"
#include "src/FilterLowPass/FilterLowPass.h"

class GenericAsservMoteurs : public Task
{
public :
    GenericAsservMoteurs(TaskType type, GenericMotor::Side side, GenericMotor::DirectionMoteur direction);
    virtual void RAZ() = 0;
    virtual void changeGains(float Kf, float Kp, float Ti, float Td) = 0;
    void byPass(int commandMotor);
    void setSpeedObjective(float speedGoal);
    float getAbsSpeedObjective() const;
    static constexpr float MAX_SPEED_MOTOR = 1200; // mm.s-1 // valeur utilisée pour plafonner les integrales de l'asserv position
    static constexpr float PRECOMMANDE_AVANCE = 1.011; // precommandes, gains statiques des blocs "asserv moteur"
    static constexpr float PRECOMMANDE_ROTATION = 130.1; // mm
    static constexpr float KF_L = 0.098; // s/mm
    static constexpr float KF_R = 0.098; // s/mm
protected:

    GenericMotor *m_motorItf;

    float m_goalSpeed; // consigne recue
    float m_Kf_motor; //0.098; // gain de feedforward moteur
    bool m_isAsservMotorByPassed = true;
};

class FakeAsservMoteursTask : public GenericAsservMoteurs
{
public :
    FakeAsservMoteursTask(TaskType type, GenericMotor::Side side, GenericMotor::DirectionMoteur direction);
    void RAZ() override {}
    void changeGains(float Kf, float Kp, float Ti, float Td) override;
protected :
    void _loop() override;
};

#endif
