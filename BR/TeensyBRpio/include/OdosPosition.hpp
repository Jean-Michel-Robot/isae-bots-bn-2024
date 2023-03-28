

/*
 * Ici on gere les odometres position, et on met à jour la position du robot
 * Une deuxieme tache gère l'envoi périodique de la position au Haut niveau
 */
#ifndef __H_ODOS
#define __H_ODOS
#include "Position2D.h"
#include "FilterLowPass.h"

class OdosPosition
{
public :
    OdosPosition();

    void setPosition(Position2D pos);
    bool isRobotBlocked(float seuil);
    void setPositionAvecRecalage();
    Position2D getRobotPosition() const;

    // le L impose les constantes en double precisions (pour une teensy)
    static constexpr float ECARTS_ODOS = 1035.095823L; //1036.02140234218L; //1026.58121393134L;     // (ticks.rad^(-1) ecart entre les 2 odos
    static constexpr float UNITS_ODOS = 12.74291667L; //12.756L;    // ticks.mm^(-1)
    static constexpr float L_R_ODOS = 1.000667458L; //1.00133324111081L; //0.999399650554833L ;  // rapport des rapports... (var en R et L)
private :
    void _loop();

    int32_t m_odoLeftCount = 0;
    int32_t m_odoRightCount = 0;
    Position2D m_robotPosition;
    double m_positionThetaOffset = 0.0;
    float m_positionThetaOdo = 0.0;

    FilterLowPass m_filterSpeedOdoL = FilterLowPass(1e-1);
    FilterLowPass m_filterSpeedOdoR = FilterLowPass(1e-1);
    int32_t m_oldOdoL = 0;
    int32_t m_oldOdoR = 0;
    float m_speedOdometerR = 0;
    float m_speedOdometerL = 0;
    unsigned long m_microsOfLastMesureSpeedOdometers = 0;


};

// class PositionSenderTask : public Task
// {
// public :
//     PositionSenderTask(TaskType type);
// private :
//     void _loop() override;
// };
// #endifuble m_positionThetaOffset = 0.0;
//     float m_positionThetaOdo = 0.0;

//     FilterLowPass m_filterSpeedOdoL = FilterLowPass(1e-1);
//     FilterLowPass m_filterSpeedOdoR = FilterLowPass(1e-1);
//     int32_t m_oldOdoL = 0;
//     int32_t m_oldOdoR = 0;
//     float m_speedOdometerR = 0;
//     float m_speedOdometerL = 0;
//     unsigned long m_microsOfLastMesureSpeedOdometers = 0;


// };

// class PositionSenderTask : public Task
// {
// public :
//     PositionSenderTask(TaskType type);
// private :
//     void _loop() override;
// };
#endif
