#include <Arduino.h>
#include <Mesure_pos.h>
#include <Moteur.h>
#include <Irsensor.h>
#include <Asserv.h>

#ifndef MACHINE_ETATS_H
#define MACHINE_ETATS_H

#define K 1
#define dt 10
#define time_global 10000
#define time_sensor 8000

#define SPEED 25
#define DISTANCE_MIN 80 // Distance minimale pour éviter un obstacle en mm

/**
 * ZONE DEPART
 */

#define DEPART_JAUNE_X 43
#define DEPART_JAUNE_Y 1741

#define DEPART_BLEU_X 43
#define DEPART_BLEU_Y 1300

/**
 * ARRIVEE
 */
#define ARRIVEE_JAUNE_1_X 1000
#define ARRIVEE_JAUNE_1_Y 300

#define ARRIVEE_JAUNE_2_X 1775
#define ARRIVEE_JAUNE_2_Y 2700

#define ARRIVEE_JAUNE_3_X 225
#define ARRIVEE_JAUNE_3_Y 2700

#define ARRIVEE_BLEU_1_X 1000
#define ARRIVEE_BLEU_1_Y 2700

#define ARRIVEE_BLEU_2_X 1775
#define ARRIVEE_BLEU_2_Y 300

#define ARRIVEE_BLEU_3_X 125
#define ARRIVEE_BLEU_3_Y 300

class Machine_etats
{
    enum Pami_State
    {
        INIT,
        MVT,
        AVOID,
        STOP,
        END,

    };

private:
    Pami_State etat;
    long m_time;
    long m_time_global;
    long m_time_sensor;

public:
    int tirette = 1;
    float pos_finit_x = ARRIVEE_BLEU_3_X;
    float pos_finit_y = ARRIVEE_BLEU_3_Y;

    float pos_init_x = DEPART_BLEU_X;
    float pos_init_y = DEPART_BLEU_Y;
    float pos_x = 0;
    float pos_y = 0;

    float angle = 0;

    Asserv *m_p_asserv;

    Mesure_pos *m_p_mesure_pos;
    Irsensor *m_p_ir_sensor;

    Machine_etats(Asserv *p_asserv, Mesure_pos *p_mesure_pos, Irsensor *p_ir_sensor);
    void setup();
    void loop();
};

#endif
