
#include "BrSM/BrSMWrapper.hpp"

#include "ROS.hpp"
#include "main_loop.hpp"

  // OrderEvent orderEvent;
  // orderEvent.order = {.x = 0, .y = 0, .theta = 0, .goalType = GoalType::FINAL};

  // send_event(orderEvent);

BrSMWrapper::BrSMWrapper() : timer(millis()) {
  BrSM::start();
}

void BrSMWrapper::loop() {

  if (millis() - timer > 20) {
    // update SM
    brUpdateEvent.currentTime = micros();
    BrSM::dispatch(brUpdateEvent);
    timer = millis();
  }
}

