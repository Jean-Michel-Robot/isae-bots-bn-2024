#ifndef __H_SETUP_LOOP
#define __H_SETUP_LOOP

#include "a_define.h"

// forward déclaration : on indique au compilateur que ces noms sont des classes, sans préciser ce qu'il y a à l'intérieur. on évite ainsi les déclarations en doublons, et les inclusion de header en cascade
class SwitchesTask;
class OdosMoteursTask;
class AsservMoteursTask;
class GenericAsservMoteurs;
class MachineAEtatAsserv;
class AsservPositionTask;
class ErrorRaiserTask;
class OdosPositionTask;
class ROSTask;
class PositionSenderTask;

// comme on définit uniquement des pointeurs, le compilateur n'a pas besoin de plus de détails
extern SwitchesTask* switchesTask;
extern AsservPositionTask* asservPositionTask;
extern MachineAEtatAsserv* machineAEtatAsservInstance;
extern OdosMoteursTask* odosMoteursTask;
extern GenericAsservMoteurs* asservMoteurLeftTask;
extern GenericAsservMoteurs* asservMoteurRightTask;
extern ErrorRaiserTask* errorRaiserTask;
extern OdosPositionTask* odosPositionTask;
extern PositionSenderTask* positionSenderTask;
extern ROSTask* rosTask;

#ifdef __SIMU__
void loop();
void setup();
#endif

#endif
