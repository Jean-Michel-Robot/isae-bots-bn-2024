/*
 * Ici on demande à la machine à état quel est objectif d'asserv (position, vitesse brute pour les moteurs ou commande brute)
 * on utilise les PID position si besoin
 * et on envoie tout à la classe d'asserv Moteur
 * */
#ifndef __H_ASSERV
#define __H_ASSERV

#include "g_Asserv_Motors.h"
#include "src/asservPID/asservPID.h"
#include "src/Task/Task.h"

class AsservPositionTask : public Task
{
public :
    AsservPositionTask(TaskType type);

    enum AsservObjectifType
    {
        OBJECTIF_POSITION,// on passe par l'asserv position
        OBJECTIF_VITESSE, // vitesse direct sur l'asserv moteur
        OBJECTIF_COMMANDE // commande direct sur les moteurs
    };

    struct AsservObjectif
    {
        AsservObjectifType type = OBJECTIF_COMMANDE;

        // cas OBJECTIF_POSITION
        float erreurAvance = 0.0; //mm
        float erreurTourne = 0.0; //rad
        float feedForwardAvancer = 0.0; //mm.s-1
        float feedForwardTourner = 0.0; //mm.s-1

        // cas OBJECTIF_VITESSE
        float speedLeft = 0.0;
        float speedRight = 0.0;

        // cas OBJECTIF_COMMANDE
        int commandeLeft = 0;
        int commandeRight = 0;
    };

    bool areGainsSet() const;
    void changeGains(float KP, float TI, float TD, float KPa, float TIa, float TDa);
    void asservRAZ();
    void setIStatusOnPID(bool isIDist,bool isIAngle);
    float getKPAngleGain() const;
private :
    void _loop() override;
    //asservPID m_asservDist = asservPID(4.17, 0.33, 0.05, 10, AsservMoteursTask::MAX_SPEED_MOTOR, AsservMoteursTask::MAX_SPEED_MOTOR); // le 4e argument est le N lie au filtrage de la derivee
    //asservPID m_asservAngle = asservPID(8, 0.1, 0.025, 10, AsservMoteursTask::MAX_SPEED_MOTOR / AsservMoteursTask::PRECOMMANDE_ROTATION , AsservMoteursTask::MAX_SPEED_MOTOR / AsservMoteursTask::PRECOMMANDE_ROTATION ); // on limite la commande en angle par rapport à la commande de distance
    asservPID m_asservDist = asservPID(30, 0.7, 0.12, 10, AsservMoteursTask::MAX_SPEED_MOTOR, AsservMoteursTask::MAX_SPEED_MOTOR); // le 4e argument est le N lie au filtrage de la derivee
    asservPID m_asservAngle = asservPID(11, 1, 0.15, 10, AsservMoteursTask::MAX_SPEED_MOTOR / AsservMoteursTask::PRECOMMANDE_ROTATION , AsservMoteursTask::MAX_SPEED_MOTOR / AsservMoteursTask::PRECOMMANDE_ROTATION ); // on limite la commande en angle par rapport à la commande de distance
    //3.0, 0.33, 0.08
    //4.0, 0.4, 0.2
};

#endif
