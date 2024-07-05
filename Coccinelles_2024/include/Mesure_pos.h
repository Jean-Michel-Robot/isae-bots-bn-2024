/**
 * @file Mesure_pos.h
 * @brief Mesure de la position et de la vitesse du robot voir readme pour plus de détail
 */

#ifndef MESURE_POS_H
#define MESURE_POS_H
#include <Arduino.h>
#include <Encodeur.h>
class Mesure_pos
{

private:
    Encodeur *m_p_encoder_R;
    Encodeur *m_p_encoder_L;
    /**
     * Facteurs proportionnels entre encodeur et roue ( a determiner empiriquement )
     * Permet de passer de la mesure de l'encodeur à la distance parcourue par la roue et à l'angle de rotation
     */
    float K_angle = 1. / 1170.32478; // TODO : regler les facteurs si on change la meca
    float K_r = 1. / 91.637717;      // TODO : regler les facteurs si on change la meca
    float K_l = 1. / 92.332506;      // TODO : regler les facteurs si on change la meca
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
     * Mesure right & left encoder
     */
    long mesure_r;
    long mesure_l;
    /**
     * Position dans le plan x , y et theta ( angle de rotation ) , mis à jour à chaque boucle
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