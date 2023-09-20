#ifndef __H_TASK_INCLUDED
#define __H_TASK_INCLUDED

#ifdef __linux__
#include <string>
#include <cmath>
#ifdef __SIMU__
#include "../../../Simulation/Arduino_defines.h"
#else
#define max(A,B) ((A)>(B)?(A):(B))
#endif
#else
#include <Arduino.h>
#endif

#include <vector>
class Task
{
public :
    enum TaskType
    {
        TaskInScheduler, // la tache est lancée en meme temps que toutes les taches du meme type, a la periode s_schedulerPeriodInMicros
        TaskAutonomous, // la tache est lancée périodiquement selon sa propre période m_periodExecuteMicros
        TaskThatNeverAutoLaunch // la tache n'est jamais lancée automatiquement
    };

    Task(TaskType type);
    void executeIfNeeded(unsigned long micros);
    void execute(){_loop();}
    void setPeriodInMicros(unsigned long micros);

    static void s_setSchedulerPeriod(unsigned long micros);
    static void s_executeAllTaskIfNeeded(unsigned long micros);
    static float s_getSchedulerWarningPeriod(unsigned long millis); // informe si un warning sur le scheduler a besoin d'etre levé. // au maximum, il y a un warning toutes les 5s
protected :
    TaskType m_typeOfTask;
    virtual void _loop() = 0;
    unsigned long m_periodExecuteMicros = 0;
    unsigned long m_lastExecuteMicros = 0;

    static std::vector<Task*> s_autonomousTaskList;
    static std::vector<Task*> s_scheduledTaskList;
    static unsigned long s_schedulerPeriodInMicros ;
    static unsigned long s_lastSchedulerExecution ;
    static float s_maxSchedulerPeriodSinceLastWarning;
    static unsigned long s_lastMillisWarningRaised;// temps du dernier warning levé
    static constexpr float SCHEDULER_TOLERANCE_RATIO = 1.2; // au dela de ce ratio en temps, le scheduler leve un warning
};


#endif
