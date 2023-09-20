#ifndef __H_ODOS_MOTOR
#define __H_ODOS_MOTOR
#include "src/Task/Task.h"
#include "src/FilterLowPass/FilterLowPass.h"
#include "src/Derivator/Derivator.h"
#include "a_define.h"

#ifdef __SIMU__
#include "../Simulation/EncoderSimu.h"
#else
#define ENCODER_OPTIMIZE_INTERRUPTS
#include "src/Encoder/Encoder.h" //https://www.pjrc.com/teensy/td_libs_Encoder.html
#endif

class OdosMoteursTask : public Task
{
public :
    OdosMoteursTask(TaskType type);

    float getSpeedL() const;
    float getSpeedR() const;
    float getAccelL() const;
    float getAccelR() const;

    static constexpr float  UNITS_ODOS_MOTOR = 105.0;    // ticks.mm^(-1) ticks par mm de roue parcourue
private :
    void _loop() override;

    float m_currentSpeedMotorL = 0.0, m_currentSpeedMotorR=0.0 , m_currentAccMotorL=0.0, m_currentAccMotorR=0.0;//mesures
//#ifdef ASSERV_MOTEURS
    FilterLowPass m_filterPreL = FilterLowPass(1e-2);
    FilterLowPass m_filterPreR = FilterLowPass(1e-2);
    FilterLowPass m_filterPostL= FilterLowPass(2e-2);
    FilterLowPass m_filterPostR= FilterLowPass(2e-2);

    Derivator_2 m_derivL = Derivator_2(2 * PI * 50, 0.5, micros(), 1e5); // on filtre à 50Hz, Q = 1/2, on sature l'acceleration à 1e5mm.s-2 (10m.s-2)
    Derivator_2 m_derivR = Derivator_2(2 * PI * 50, 0.5, micros(), 1e5);;

    Encoder m_knobLeft = Encoder(18, 17);
    Encoder m_knobRight = Encoder(20,19);
//#endif
};

#endif
