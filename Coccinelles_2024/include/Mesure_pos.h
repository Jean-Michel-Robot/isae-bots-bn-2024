#include <Arduino.h>
#include <Encodeur.h>

#ifndef MESURE_POS_H
#define MESURE_POS_H

class Mesure_pos
{

private:
    Encodeur *m_p_encoder_R;
    Encodeur *m_p_encoder_L;
    /**
     * Facteurs proportionnels entre encodeur et roue ( a determiner empiriquement )
     */
    float K_angle = 1. / 1170.32478;
    float K_r = 1. / 91.637717;
    float K_l = 1. / 92.332506;
    /**
     * temps entre deux mesures
     */
    long dt;

    /**
     * temps mis a jour à chaque boucle
     */
    long m_time;

public:
    void setup();
    void loop();
    Mesure_pos(Encodeur *p_encodeur_r, Encodeur *p_encodeur_l);
    /**
     * Mesure right & left
     */
    long mesure_r;
    long mesure_l;
    /**
     * Position dans le plan x , y et theta ( angle de rotation )
     */
    float position_x;
    float position_y;
    float position_theta;

    /**
     * Vitesse selon x , y et theta
     */
    float Vitesse_x;
    float Vitesse_y;
    float Vitesse_theta;

    /**
     * vitesse des roues droites et gauche
     */

    float vitesse_r;
    float vitesse_l;
};

#endif