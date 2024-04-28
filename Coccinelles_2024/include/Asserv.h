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
    long m_time;

public:
    Asserv(Moteur *p_moteur_r, Moteur *p_moteur_l, Mesure_pos *p_mesure_pos);
    asservPID m_asservPID_r;
    asservPID m_asservPID_l;
    asservPID m_asservPID_angle;

    Moteur *m_p_moteur_r;
    Moteur *m_p_moteur_l;
    Mesure_pos *m_p_mesure_pos;
    void setup();
    void loop();
    void asservissement(float vitesse_l_consign, float vitesse_r_consigne);
    void asservissement_angle(float theta_consigne);
    void asserv_global(float vitesse_l_consign, float vitesse_r_consigne, float theta_consigne);
};

#endif