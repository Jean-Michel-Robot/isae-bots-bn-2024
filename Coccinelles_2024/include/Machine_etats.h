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

#define SPEED 20
#define DISTANCE_MIN 30 // Distance minimale pour éviter un obstacle en mm

#define POS_INIT_X 0
#define POS_INIT_Y 0

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
    float pos_finit_x = 40;
    float pos_finit_y = 1000;

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
