/*
 * Ici on détecte si une roue patine en comparant les odos moteurs et les odos positions
 */
#ifndef __H_ERROR_MANAGER
#define __H_ERROR_MANAGER
#include "src/Task/Task.h"
#include "src/FilterLowPass/FilterLowPass.h"
#include "src/Timer/Timer.h"
#include "src/Position2D/Position2D.h"
void loopErrorManager();

class ErrorRaiserTask : public Task
{
public :
    ErrorRaiserTask(TaskType type);
private :
    void _loop() override;
    // le seuil de derapage est ajusté en fonction de la vitesse de la roue (on prend le plus grand des 2 seuils)
    static constexpr float SEUIL_DERAPAGE = 50; //mm.s-1 // seuil minimum
    static constexpr float SEUIL_DERAPAGE_RAPPORT = 0.2; // 2e seuil : vitesse de la roue * ce facteur
    static constexpr float TIMEMAX_DERAPAGE = 1.0; //s // duree de derapage autorisée
    Timer m_timerOutPatinage = Timer(TIMEMAX_DERAPAGE); // timeout patinage de roue

    float m_speedLEstimatedWithPosition = 0.0; // vitesse estimee de la roue gauche par l'odométrie position
    float m_speedREstimatedWithPosition = 0.0; // vitesse estimee de la roue droite par l'odométrie position
    float m_speedLEstimatedWithMotors = 0.0; // vitesse de la roue gauche avec l'odométrie moteur
    float m_speedREstimatedWithMotors = 0.0; // vitesse de la roue gauche avec l'odométrie moteur
    Position2D m_lastPos;
    unsigned long m_lastTime = 0;
    FilterLowPass m_filterOmega = FilterLowPass(0.1);
    FilterLowPass m_filterVit = FilterLowPass(0.1);
    FilterLowPass m_filterLDelay = FilterLowPass(0.12); // les durees des filtres sont differentes pour essayer de les resynchroniser, néanmoins il reste un décalage entre les deux moyens de mesure
    FilterLowPass m_filterRDelay = FilterLowPass(0.12); // d'où les seuils assez permissifs
};

#endif
