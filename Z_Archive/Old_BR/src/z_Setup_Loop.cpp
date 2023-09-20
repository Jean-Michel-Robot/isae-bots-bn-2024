#include "a_define.h"
#include "a_Led_RGB.h"
#include "c_switchs.h"
#include "e_odos_motor.h"
#include "g_Asserv_Motors.h"
#include "i_etat_asserv.h"
#include "k_Asserv.h"
#include "l_error_manager.h"
#include "p_Odos.h"
#include "u_ROS.h"
#include "z_Setup_Loop.h"
#include "src/Task/Task.h"
#include "g_Asserv_Motors_Generic.h"

SwitchesTask* switchesTask = NULL;
OdosMoteursTask* odosMoteursTask = NULL;
GenericAsservMoteurs* asservMoteurLeftTask = NULL;
GenericAsservMoteurs* asservMoteurRightTask = NULL;
MachineAEtatAsserv* machineAEtatAsservInstance = NULL;
AsservPositionTask* asservPositionTask = NULL;
OdosPositionTask* odosPositionTask = NULL;
PositionSenderTask* positionSenderTask = NULL;
ROSTask* rosTask = NULL;
void setup()
{
    // on créé d'abord les instances, qui sont utilisées par les taches mais qui ne s'executent pas d'elles meme
    machineAEtatAsservInstance = new MachineAEtatAsserv();

    // on fixe la période d'execution des taches principales
    Task::s_setSchedulerPeriod(2000); // 2ms

    // on ajoute toutes les taches principales. elles seront toutes executées les unes après les autres, dans l'ordre donné, à la période fixée
    switchesTask = new SwitchesTask(Task::TaskInScheduler);
    odosMoteursTask = new OdosMoteursTask(Task::TaskInScheduler);
#ifdef ASSERV_MOTEURS
    asservMoteurLeftTask = new AsservMoteursTask(Task::TaskInScheduler, GenericMotor::LEFT,GenericMotor::FORWARD);
    asservMoteurRightTask = new AsservMoteursTask(Task::TaskInScheduler, GenericMotor::RIGHT, GenericMotor::REVERSE);
#else
    asservMoteurLeftTask = new FakeAsservMoteursTask(Task::TaskInScheduler, GenericMotor::LEFT,GenericMotor::FORWARD);
    asservMoteurRightTask = new FakeAsservMoteursTask(Task::TaskInScheduler, GenericMotor::RIGHT, GenericMotor::REVERSE);
#endif
    asservPositionTask = new AsservPositionTask(Task::TaskInScheduler);
    rosTask = new ROSTask(Task::TaskInScheduler);

    // on ajoute les taches indépendantes, executées à une frequence propre
    odosPositionTask = new OdosPositionTask(Task::TaskAutonomous);
    odosPositionTask->setPeriodInMicros(200); // 200 us

    positionSenderTask = new PositionSenderTask(Task::TaskAutonomous);
    positionSenderTask->setPeriodInMicros(20000); // 20 ms

}

void loop()
{    
    Task::s_executeAllTaskIfNeeded(micros()); // on execute les taches qui ont besoin de l'etre
    float maxTimeOfScheduler = Task::s_getSchedulerWarningPeriod(millis());
    if(maxTimeOfScheduler > 0)
        rosTask->logPrint("Warning : scheduler period were too long ,period " + String(maxTimeOfScheduler));
}
