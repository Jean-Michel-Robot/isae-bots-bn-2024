/**
 * @file Asserv.cpp
 * @brief Asservissement en vitesse et en angle du robot
 *
 */
// TODO REGLER TOUS LES GAINS AVANT LA COUPE
#ifndef ASSERV_H
#define ASSERV_H

#include <Mesure_pos.h>
#include <Arduino.h>
#include <asservPID.h>
#include <Moteur.h>

class Asserv
{
private:
    // Facteurs pour passer d'une vitesse en cm/s à une vitesse entre 0 et 255
    float Kmot_r = 20;
    float Kmot_l = 20;
    float Kmot_angle = 50;
    long dt = 10;
    long m_time; // Variable de temps ou on stocke le temps actuel

public:
    /**
     * @brief Constructeur classe asserv , prend en argument les deux moteurs du robot ainsi que sa position
     */
    Asserv(Moteur *p_moteur_r, Moteur *p_moteur_l, Mesure_pos *p_mesure_pos);
    asservPID m_asservPID_r;     // asserv pour la roue droite
    asservPID m_asservPID_l;     // asserv pour la roue gauche
    asservPID m_asservPID_angle; // asserv pour l'angle

    Moteur *m_p_moteur_r; // moteur droit
    Moteur *m_p_moteur_l; // moteur gauche
    Mesure_pos *m_p_mesure_pos;
    void setup(); // Initialisation des asservissements
    void loop();  // Boucle d'asservissement, ne sert que pour test l'asserv
    /**
     * @brief Asservissement en vitesse des roues
     */
    void asservissement(float vitesse_l_consign, float vitesse_r_consigne);
    /**
     * @brief Asservissement en angle
     */
    void asservissement_angle(float theta_consigne);
    /**
     * @brief Asservissement global en vitesse et en angle
     */
    void asserv_global(float vitesse_l_consign, float vitesse_r_consigne, float theta_consigne);
};

#endif