#ifndef __H_ASSERV_MOTORS
#define __H_ASSERV_MOTORS
#include "src/Task/Task.h"
#include "src/asservPID/asservPID.h"
#include "src/FilterLowPass/FilterLowPass.h"

#include "g_Asserv_Motors_Generic.h"


class AsservMoteursTask : public GenericAsservMoteurs
{
public :
    AsservMoteursTask(TaskType type, GenericMotor::Side side, GenericMotor::DirectionMoteur direction);
    void RAZ() override;
    void changeGains(float Kf, float Kp, float Ti, float Td) override; // override the GenericAsserv method
    static constexpr float MAX_SPEED_MOTOR = 1200; // mm.s-1 // valeur utilisée pour plafonner les integrales de l'asserv position
    static constexpr float PRECOMMANDE_AVANCE = 1.011; // precommandes, gains statiques des blocs "asserv moteur"
    static constexpr float PRECOMMANDE_ROTATION = 143.1;
private :
    void _loop() override;

    asservPID m_asservMotor = asservPID(0.095, 0.04, 0.01, 15, CytronMotorBR::MAX_COMMAND_MOTOR, CytronMotorBR::MAX_COMMAND_MOTOR); // on bride les PID a la commande max des moteurs;
    FilterLowPass m_filterComD = FilterLowPass(1e-2);

};

#endif
