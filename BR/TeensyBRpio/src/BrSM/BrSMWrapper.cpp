
#include "BrSM/BrSMWrapper.hpp"

#include "BrSM/BrSM.hpp"
#include "ROS.hpp"
#include "main_loop.hpp"

  // OrderEvent orderEvent;
  // orderEvent.order = {.x = 0, .y = 0, .theta = 0, .goalType = GoalType::FINAL};

  // send_event(orderEvent);

BrSMWrapper::BrSMWrapper() {


  brSM = BrSM();  // stack memory (instanciation without new keyword)

  // beginRampEvent.t0 = 0.0;
  // goalSpeedChangeEvent.newSpeed = 0.0;
  // updateEvent.currentTime = 0.0;

  // brSM.setupTrajectory();
  brUpdateEvent.currentTime = micros();

  brSM.start();

}

void BrSMWrapper::loop() {

  // update SM
  brUpdateEvent.currentTime = micros();
  brSM.send_event(brUpdateEvent);
}

