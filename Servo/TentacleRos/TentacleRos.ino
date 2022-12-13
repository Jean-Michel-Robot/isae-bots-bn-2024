#include "TentacleRos.h"

#define SWITCH_PIN 2
#define SWITCH_TAU 0.05
#define SWITCH_THRESHOLD 0.5

#define PUMP_PIN 12

#define SERVO_PIN 4

#define SWITCH_PIN_2 3
#define PUMP_PIN_2 13
#define SERVO_PIN_2 5

int servoPositions[] = {135, 55, 45, 40};

#define TIME_IN_CONTACT 0.1 // s
#define TIME_LOST_CONTACT 0.5 // s

#define TIME_GRABBING 0.8 // s

#define TIME_LOST_CONTACT_BEFORE_PUMP_OFF_RELEASING 0.1 // s

#define TIME_BEFORE_PUMP_OFF_RELEASING_FROM_MID 0.3 // s
#define TIME_RELEASING_FROM_MID 2.0 // s

#define TIME_BEFORE_PUMP_OFF_RELEASING_FROM_HIGH 0.8 // s
#define TIME_RELEASING_FROM_HIGH 2.5 // s

#define ROS_NB_TENTACLES 2

#define TIME_PUB_STATES 0.5 // s

ros::NodeHandle nh;

TentacleStateMachine *tentacleStateMachineFR = new TentacleStateMachine("TFR", PUMP_PIN,
    SWITCH_PIN, SWITCH_TAU, SWITCH_THRESHOLD,
    "SFR", SERVO_PIN, servoPositions,
    TIME_IN_CONTACT, TIME_LOST_CONTACT, TIME_GRABBING,
    TIME_LOST_CONTACT_BEFORE_PUMP_OFF_RELEASING,
    TIME_BEFORE_PUMP_OFF_RELEASING_FROM_MID, TIME_RELEASING_FROM_MID,
    TIME_BEFORE_PUMP_OFF_RELEASING_FROM_HIGH, TIME_RELEASING_FROM_HIGH,
    FR);

TentacleStateMachine *tentacleStateMachineMR = new TentacleStateMachine("TMR", PUMP_PIN_2,
    SWITCH_PIN_2, SWITCH_TAU, SWITCH_THRESHOLD,
    "SMR", SERVO_PIN_2, servoPositions,
    TIME_IN_CONTACT, TIME_LOST_CONTACT, TIME_GRABBING,
    TIME_LOST_CONTACT_BEFORE_PUMP_OFF_RELEASING,
    TIME_BEFORE_PUMP_OFF_RELEASING_FROM_MID, TIME_RELEASING_FROM_MID,
    TIME_BEFORE_PUMP_OFF_RELEASING_FROM_HIGH, TIME_RELEASING_FROM_HIGH,
    MR);

TentacleStateMachine *tentacleStateMachineArray[] = {tentacleStateMachineFR, tentacleStateMachineMR};

//TentacleStateMachine *tentacleStateMachineArray[] = {tentacleStateMachineFR};

TentacleRos tentacleRos = TentacleRos(&nh, ROS_NB_TENTACLES, tentacleStateMachineArray, TIME_PUB_STATES);

void setup() {
    nh.initNode();

    for (int i = 0; i < ROS_NB_TENTACLES; i++) {
        tentacleStateMachineArray[i]->setup();
    }

    tentacleRos.setup();

}

void loop() {
    for (int i = 0; i < ROS_NB_TENTACLES; i++) {
        tentacleStateMachineArray[i]->loop();
    }
    
    tentacleRos.loop();

	nh.spinOnce();
}
