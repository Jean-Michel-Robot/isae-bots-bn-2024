#include "ros/Logger.hpp"
#include "utils/clock.h"
#include "defines.hpp"
#include "geometry/Position2D.h"
#include "feedback/PositionFeedback.hpp"
#include "trajectories/Trajectory.hpp"
#include "state_machine/BrSM.hpp"
#include "state_machine/RampSM.hpp"
#include "Asserv.hpp"

#include <stdlib.h>

long Logger::loop_timer = 0;

float Logger::m_arrayOfValues[Logger::NbOfFields];

void Logger::setFieldValue(float value, Logger::LogField fieldName)
{
  if (fieldName < NbOfFields)
  {
    m_arrayOfValues[fieldName] = value;
  }
}

float *Logger::getArrayOfValues()
{
  setFieldValue(micros() * 1e-6, currentTime);
  return m_arrayOfValues;
}

void Logger::loop()
{
  // Log update
  if (millis() - loop_timer > LOG_PERIOD)
  {

    Logger::setFieldValue(millis(), Logger::currentTime);

    Position2D robotPos = PositionFeedback::instance().getRobotPosition();
    Logger::setFieldValue(robotPos.x, Logger::robotPosX);
    Logger::setFieldValue(robotPos.y, Logger::robotPosY);
    Logger::setFieldValue(robotPos.theta, Logger::robotPosTheta);

    Position2D goalPos = BrSM::getCurrentGoalPos();
    Logger::setFieldValue(goalPos.x, Logger::goalPointPosX);
    Logger::setFieldValue(goalPos.y, Logger::goalPointPosY);
    Logger::setFieldValue(goalPos.theta, Logger::goalPointPosTheta);

    if (!BrSM::currentTrajectory)
    {
      Logger::setFieldValue(0, Logger::goalPointSpeedX);
      Logger::setFieldValue(0, Logger::goalPointSpeedY);

      Logger::setFieldValue(0.0, Logger::goalSpeedLinear);
      Logger::setFieldValue(0.0, Logger::goalSpeedAngular);
    }
    else
    {

      float *goalSpeed = BrSM::currentTrajectory->getTrajectoryAbsoluteSpeed();
      Logger::setFieldValue(goalSpeed[0], Logger::goalPointSpeedX);
      Logger::setFieldValue(goalSpeed[1], Logger::goalPointSpeedY);

      Logger::setFieldValue(BrSM::currentTrajectory->s, Logger::trajectoryS);

      if (BrSM::currentTrajectory->trajectoryType == TrajectoryType::TRAJ_LINEAR)
      {
        Logger::setFieldValue(BrSM::currentTrajectory->goalSpeed, Logger::goalSpeedLinear);
        Logger::setFieldValue(0.0, Logger::goalSpeedAngular);
      }
      else if (BrSM::currentTrajectory->trajectoryType == TrajectoryType::TRAJ_ROTATION)
      {
        Logger::setFieldValue(0.0, Logger::goalSpeedLinear);
        Logger::setFieldValue(BrSM::currentTrajectory->goalSpeed, Logger::goalSpeedAngular);
      }
    }

    Logger::setFieldValue(Asserv::instance().error[0], Logger::asservErrorX);
    Logger::setFieldValue(Asserv::instance().error[1], Logger::asservErrorY);

    Logger::setFieldValue(Asserv::instance().cmd_v, Logger::commandV);
    Logger::setFieldValue(Asserv::instance().cmd_omega, Logger::commandOmega);

    Logger::setFieldValue(Asserv::instance().m_rightWheelSpeed, Logger::commandeMotorR);
    Logger::setFieldValue(Asserv::instance().m_leftWheelSpeed, Logger::commandeMotorL);

    if (!BrSM::currentTrajectory)
    {
      Logger::setFieldValue(0.0, Logger::rampSpeed);
      Logger::setFieldValue(0.0, Logger::rampState);
    }
    else
    {
      Logger::setFieldValue(RampSM::getCurrentSpeed(), Logger::rampSpeed);
      Logger::setFieldValue(RampSM::getCurrentState(), Logger::rampState);
    }

    Logger::setFieldValue(BrSM::getCurrentState(), Logger::BrState);
    loop_timer = millis();
  }
}