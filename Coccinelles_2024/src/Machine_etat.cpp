#include <Machine_etats.h>
#include <Arduino.h>
#include <cmath>

Machine_etats::Machine_etats(Asserv *p_asserv, Mesure_pos *p_mesure_pos, Irsensor *p_ir_sensor)
{
    m_p_asserv = p_asserv;
    m_p_mesure_pos = p_mesure_pos;
    m_p_ir_sensor = p_ir_sensor;
}

void Machine_etats::setup()
{
    pinMode(34, INPUT);
    etat = INIT;
    m_time = millis();
    m_time_global = millis();
}

void Machine_etats::loop()
{
    if (millis() - m_time >= dt)
    {
        if (millis() - m_time_global >= time_global)
        {
            m_p_asserv->asserv_global(0, 0, 0);
            etat = END;
        }
        // Lire l'état de la tirette
        tirette = digitalRead(34);
        Serial.println(tirette);
        switch (etat)
        {
        case INIT:
            if (tirette == 0)
            {
                m_time_global = millis();
                etat = MVT;
            }
            else
            {
                etat = INIT;
            }
            break;
        case MVT:
            pos_x = m_p_mesure_pos->position_x + pos_init_x;
            pos_y = m_p_mesure_pos->position_y + pos_init_y;

            angle = atan2(pos_y - pos_finit_y, pos_finit_x - pos_x);
            m_p_asserv->asserv_global(SPEED, SPEED, angle);
            if (abs(pos_finit_x - pos_x) < 5 && abs(pos_finit_y - pos_y) < 5)
            {
                etat = END;
            }
            if ((m_p_ir_sensor->m_minimum_distance) <= DISTANCE_MIN)
            {
                etat = STOP;
            }
            break;
        case AVOID:
            // Faites tourner le robot dans une direction spécifique
            // (par exemple, en ajustant l'angle)
            angle += PI / 2; // Tournez de 90 degrés à droite
            m_p_asserv->asserv_global(SPEED, SPEED, angle);

            // Si l'obstacle n'est plus détecté, revenez à l'état MVT
            if ((m_p_ir_sensor->m_minimum_distance) > DISTANCE_MIN)
            {
                etat = MVT;
            }
            break;

        case STOP:
            m_p_asserv->asserv_global(0, 0, angle);
            if ((m_p_ir_sensor->m_minimum_distance) > DISTANCE_MIN)
            {
                etat = MVT;
            }
            break;

        case END:
            m_p_asserv->asserv_global(0, 0, angle);
            break;
        }
        m_time = millis();
    }
}