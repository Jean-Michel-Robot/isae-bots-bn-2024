#include <Asserv.h>
#include <Moteur.h>
#include <Arduino.h>
#include <Mesure_pos.h>
/**
 * @file Asserv.cpp
 * @brief Asservissement en vitesse et en angle du robot
 * 
*/
Asserv::Asserv(Moteur *p_moteur_r, Moteur *p_moteur_l, Mesure_pos *p_mesure_pos) : m_asservPID_r(1, 0.1, 0, 255, 5), //TODO : regler l'asservissement
                                                                                   m_asservPID_l(1, 0.1, 0, 255, 5), 
                                                                                   m_asservPID_angle(50, 0, 0, 255, 5)
{
    m_p_mesure_pos = p_mesure_pos;
    m_p_moteur_l = p_moteur_l;
    m_p_moteur_r = p_moteur_r;
}

void Asserv::asservissement(float vitesse_l_consigne, float vitesse_r_consigne)
{
    float erreur_l = vitesse_l_consigne - m_p_mesure_pos->vitesse_l;
    float erreur_r = vitesse_r_consigne - m_p_mesure_pos->vitesse_r;
    float output_l = m_asservPID_l.computeOutput(erreur_l, micros());
    float output_r = m_asservPID_r.computeOutput(erreur_r, micros());
    Serial.println("output");
    Serial.printf("serial_d = %f \n", output_r * Kmot_r);
    m_p_moteur_l->set_speed(output_l * Kmot_l);
    m_p_moteur_r->set_speed(output_r * Kmot_r);
}

void Asserv::asservissement_angle(float theta_consigne)
{
    float erreur = theta_consigne - m_p_mesure_pos->position_theta;
    erreur = fmod(erreur, 2 * PI);
    if (erreur > PI)
    {
        erreur -= 2 * PI;
    }
    else if (erreur < -PI)
    {
        erreur += 2 * PI;
    }
    float output = m_asservPID_angle.computeOutput(erreur, micros());
    Serial.println(output);
    Serial.print(erreur);
    m_p_moteur_l->set_speed(-output * Kmot_angle);
    m_p_moteur_r->set_speed(output * Kmot_angle);
}

void Asserv::setup()
{
    m_time = micros();
    m_asservPID_l.RAZ(micros());
    m_asservPID_r.RAZ(micros());
    m_asservPID_angle.RAZ(micros());
}

void Asserv::asserv_global(float vitesse_l_consigne, float vitesse_r_consigne, float theta_consigne)
{
    float erreur_l = vitesse_l_consigne - m_p_mesure_pos->vitesse_l;
    float erreur_r = vitesse_r_consigne - m_p_mesure_pos->vitesse_r;
    float output_l = m_asservPID_l.computeOutput(erreur_l, micros());
    float output_r = m_asservPID_r.computeOutput(erreur_r, micros());

    float erreur_theta = theta_consigne - m_p_mesure_pos->position_theta;
    erreur_theta = fmod(erreur_theta, 2 * PI);
    if (erreur_theta > PI)
    {
        erreur_theta -= 2 * PI;
    }
    else if (erreur_theta < -PI)
    {
        erreur_theta += 2 * PI;
    }
    float output_theta = m_asservPID_angle.computeOutput(erreur_theta, micros());

    m_p_moteur_l->set_speed(output_theta * Kmot_angle + output_l * Kmot_l);
    m_p_moteur_r->set_speed(output_r * Kmot_r - output_theta * Kmot_angle);
}
void Asserv::loop()
{
    if (micros() - m_time >= 1e4)
    {
        asserv_global(0, 0, PI / 2);
        m_time = micros();
    }
}