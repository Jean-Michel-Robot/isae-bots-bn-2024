
#ifndef MACHINE_ETATS_H
#define MACHINE_ETATS_H

#include <Arduino.h>
#include <Mesure_pos.h>
#include <Moteur.h>
#include <Irsensor.h>
#include <Asserv.h>
#include <Define_map.h>

#define K 1
#define dt 10
#define time_global 10000
#define time_sensor 8000

#define SPEED 25        // Vitesse en cm/s 25 est la vitesse max des moteurs
#define DISTANCE_MIN 80 // Distance minimale pour éviter un obstacle en mm

class Machine_etats
{
    enum Pami_State
    {
        INIT,
        MVT,
        AVOID, // Etat pour éviter un obstacle (non implémenté)
        STOP,
        END,

    };

private:
    Pami_State etat;
    long m_time;
    long m_time_global;
    long m_time_sensor;

public:
    int tirette = 1; // TODO Etat par défaut de la tirette , CHANGER SI NECESSAIRE
    /**
     * Stratégie de déplacement  , Position de départ et d'arrivée
     */
    float pos_finit_x = ARRIVEE_BLEU_3_X; // TODO : A MODIFIER en foction de la stratégie
    float pos_finit_y = ARRIVEE_BLEU_3_Y; // TODO : A MODIFIER en foction de la stratégie

    float pos_init_x = DEPART_BLEU_X; // TODO : A MODIFIER en foction de la stratégie
    float pos_init_y = DEPART_BLEU_Y; // TODO : A MODIFIER en foction de la stratégie
    // Position actualisé du robot
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
