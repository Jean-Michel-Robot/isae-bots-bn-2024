#ifdef ARDUINO
#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdlib.h>

#include "utils/Led.hpp"

#include "utils/clock.h"

#include "motors.hpp"
#include "feedback/PositionFeedback.hpp"

#include "logging.h"
#include "ros/Logger.hpp"
#include "ros/ROS.hpp"

#include "trajectories/LinearTrajectory.hpp"
#include "trajectories/RotationTrajectory.hpp"
#include "Asserv.hpp"
#include "state_machine/BrSMWrapper.hpp"
#include "state_machine/BrSM.hpp"
#include "state_machine/RampSM.hpp"

#define DEBUG_MODE true

BlinkLED *p_blink = NULL;

void setup()
{
    Serial.begin(115200);
    set_microros_serial_transports(Serial);

    motors_init();

    p_blink = new BlinkLED();
}

uint32_t loop_timer_debug = 0.0;

void loop()
{

    PositionFeedback::instance().loop();
    p_blink->loop();
    BrSMWrapper::instance().loop();

    Logger::loop();
    ROS::instance().loop();

    // Commands for debugging
    if (DEBUG_MODE && millis() - loop_timer_debug > LOG_PERIOD && Serial.available())
    {
        char c = Serial.read();

        if (c == 't')
        {
            Serial.println("Test input");
        }

        else if (c == 'r')
        {
            log(INFO, "Received get ready event");

            BrGetReadyEvent brGetReadyEvent;
            BrSM::dispatch(brGetReadyEvent);
        }

        else if (c == 'o')
        {
            Serial.println("Received order request");

            OrderEvent orderEvent;
            orderEvent.order.x = 0.5;
            orderEvent.order.y = 0.0;
            orderEvent.order.theta = 0.0;
            orderEvent.order.goalType = GoalType::TRANS; // on essaie direct le depl linéaire

            BrSM::dispatch(orderEvent);
        }

        else if (c == 's')
        {
            //Serial.println("Send goal speed change event of 0.5");

            GoalSpeedChangeEvent goalSpeedChangeEvent;
            goalSpeedChangeEvent.newSpeed = 0.5;

            RampSM::dispatch(goalSpeedChangeEvent);
        }
        else if (c == 'd')
        {
            //Serial.println("Send goal speed change event of 0.1");

            GoalSpeedChangeEvent goalSpeedChangeEvent;
            goalSpeedChangeEvent.newSpeed = 0.1;

            RampSM::dispatch(goalSpeedChangeEvent);
        }

        else if (c == 'b')
        {
            //Serial.println("Send emergency brake event");

            EmergencyBrakeEvent emergencyBrakeEvent;

            RampSM::dispatch(emergencyBrakeEvent);
        }

        else if (c == 'e')
        {
            //Serial.println("Send end ramp event");

            EndRampEvent endRampEvent;

            RampSM::dispatch(endRampEvent);
        }

        loop_timer_debug = millis();
    }
}

#endif