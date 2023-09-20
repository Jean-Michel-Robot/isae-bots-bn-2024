#include "Task.h"

Task::Task(TaskType type):m_typeOfTask(type)
{
    switch(m_typeOfTask)
    {
        case TaskInScheduler:
            s_scheduledTaskList.push_back(this);
        break;
        case TaskAutonomous:
            s_autonomousTaskList.push_back(this);
        break;
        case TaskThatNeverAutoLaunch:
        break;
    }
}

void Task::executeIfNeeded(unsigned long micros)
{
    if((micros-m_lastExecuteMicros) >= m_periodExecuteMicros && m_periodExecuteMicros > 0)
    {
        _loop();
        m_lastExecuteMicros = micros;
    }
}

void Task::setPeriodInMicros(unsigned long micros)
{
    m_periodExecuteMicros = micros;
}

unsigned long Task::s_schedulerPeriodInMicros = 2000;
unsigned long Task::s_lastSchedulerExecution = 1000; // we let some time for the boot
std::vector<Task*> Task::s_autonomousTaskList;
std::vector<Task*> Task::s_scheduledTaskList;
float Task::s_maxSchedulerPeriodSinceLastWarning = 0.0;
unsigned long Task::s_lastMillisWarningRaised = 0;
void Task::s_setSchedulerPeriod(unsigned long micros)
{
    s_schedulerPeriodInMicros = micros;
}

void Task::s_executeAllTaskIfNeeded(unsigned long micros)
{
    for(Task* task:s_autonomousTaskList)
    {
        task->executeIfNeeded(micros);
    }
    if(micros - s_lastSchedulerExecution >= s_schedulerPeriodInMicros)
    {
        s_maxSchedulerPeriodSinceLastWarning = max(micros - s_lastSchedulerExecution,s_maxSchedulerPeriodSinceLastWarning);
        s_lastSchedulerExecution = micros;
        for(Task* scheduledTask: s_scheduledTaskList)
        {
            scheduledTask->execute();
        }
    }
}

float Task::s_getSchedulerWarningPeriod(unsigned long millis)
{

    if(s_maxSchedulerPeriodSinceLastWarning> s_schedulerPeriodInMicros*SCHEDULER_TOLERANCE_RATIO && millis > s_lastMillisWarningRaised + 5000)
    {
        s_lastMillisWarningRaised = millis;
        float ret = s_maxSchedulerPeriodSinceLastWarning;
        s_maxSchedulerPeriodSinceLastWarning = 0;
        return ret * 1e-6;
    }
    return -1;
}
