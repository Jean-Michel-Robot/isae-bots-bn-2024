#include <Mesure_pos.h>

Mesure_pos::Mesure_pos(Encodeur *p_encodeur_r, Encodeur *p_encodeur_l)
{
    m_p_encoder_L = p_encodeur_l;
    m_p_encoder_R = p_encodeur_r;
}

void Mesure_pos::setup()
{
    dt = 10;
    m_p_encoder_L->setup();
    m_p_encoder_R->setup();
    position_theta = 0;
    position_x = 0;
    position_y = 0;
    mesure_r = m_p_encoder_R->mesure();
    mesure_l = m_p_encoder_L->mesure();
    m_time = millis();
}
void Mesure_pos::loop()
{
    if (millis() - m_time >= dt)
    {
        float position_l = (m_p_encoder_L->mesure() - mesure_l);
        float position_r = (m_p_encoder_R->mesure() - mesure_r);
        // Positions
        position_theta += (position_l - position_r) * K_angle;
        position_x += ((position_l * K_l + position_r * K_r) / 2) * cos(position_theta);
        position_y += ((position_l * K_l + position_r * K_r) / 2) * sin(position_theta);

        // Vitesse
        Vitesse_x = (((position_l * K_l + position_r * K_r) / 2) * cos(position_theta) / dt) * 1000; // En cm metre  par seconde
        Vitesse_y = (((position_l * K_l + position_r * K_r) / 2) * sin(position_theta) / dt) * 1000; // En cm metre  par seconde
        Vitesse_theta = ((position_l - position_r) * K_angle / dt) * 1000;                           // En cm radian par seconde
        // Vitesse roue droite et gauche (en cm/s)

        vitesse_l = (position_l / dt * 1000) * K_l;
        vitesse_r = (position_r / dt * 1000) * K_r;

        /**
         * Affichage des valeurs , a decommenter si on veut debug , 
         * NE PAS OUBLIER DE COMMENTER DANS LE CODE FINAL SINON LE TERMINAL SERIE INTERFERE AVEC L'ASSERVISSEMENT ET CA FAIT NIMP
        */
       
        // Serial.println("Position_x = " + String(position_x));
        // Serial.println("Position_y = " + String(position_y));
        Serial.println("Position_theta = " + String(position_theta));
        // Serial.println("Vitesse_x = " + String(Vitesse_x));
        // Serial.println("Vitesse_y = " + String(Vitesse_y));
        // Serial.println("Vr=" + String(vitesse_r));
        // Serial.println("Vl=" + String(vitesse_l));

        // Serial.println("Mesure_r = " + String(mesure_r));
        // Serial.println("Mesure_l= " + String(mesure_l));
        //    Maj des mesures et temps ;
        mesure_r = m_p_encoder_R->mesure();
        mesure_l = m_p_encoder_L->mesure();
        m_time = millis();
    }
}