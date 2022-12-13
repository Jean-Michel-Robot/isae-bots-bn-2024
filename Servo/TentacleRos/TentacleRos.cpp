/*
Allows for the creation of TentacleRos objects which manages the ros communication for the 6 tentacles
*/

#include "TentacleRos.h"

int TentacleRos::nbTentacles = 0;
TentacleStateMachine **TentacleRos::tentacleStateMachineArray = NULL;

TentacleRos::TentacleRos(ros::NodeHandle *nh, int nbTentacles, TentacleStateMachine **tentacleStateMachineArray, float timePubStates)
    : timerPubStates(timePubStates),
    subTentacleOrders("tentacleOrders", tentacleOrdersCb),
    pubTentacleStates("tentacleStates", &tentacleState),
    pubTentacleReleaseFbs("tentacleReleaseFbs", &tentacleReleaseFb) {

    this->m_nh = nh;

    this->nbTentacles = nbTentacles;
    this->tentacleStateMachineArray = tentacleStateMachineArray;

}

void TentacleRos::tentacleOrdersCb(const std_msgs::Int16 &orderMsg) {
    int tentacleId = orderMsg.data / 10;
    int tentacleOrderId = orderMsg.data % 10;

    if (tentacleId < nbTentacles) {
        switch (tentacleOrderId) {
            case ORDER_IDLE:
                tentacleStateMachineArray[tentacleId]->stateDoTransition(STATE_IDLE);
            break;
            case ORDER_GRAB:
                tentacleStateMachineArray[tentacleId]->stateDoTransition(STATE_GRABBING);
            break;
            case ORDER_RELEASE:
                tentacleStateMachineArray[tentacleId]->stateDoTransition(STATE_RELEASING);
            break;
            case ORDER_CARRY:
                tentacleStateMachineArray[tentacleId]->stateDoTransition(STATE_CARRYING);
            break;
            case ORDER_CARRY_HIGH:
                tentacleStateMachineArray[tentacleId]->stateDoTransition(STATE_CARRYING_HIGH);
            break;
            case ORDER_END:
                tentacleStateMachineArray[tentacleId]->stateDoTransition(STATE_END);
            break;
        }
    }

}

void TentacleRos::publishTentacleState(int id) {
    //tentacleState.data = id;
    tentacleState.data = (int) id * 10 + (int) tentacleStateMachineArray[id]->getCurrentState();
    pubTentacleStates.publish(&tentacleState);
}

void TentacleRos::publishReleaseFb(int id, int isReleaseSuccessful) {
    tentacleReleaseFb.data = (int) id * 10 + (int) isReleaseSuccessful;
    pubTentacleReleaseFbs.publish(&tentacleReleaseFb);
}

void TentacleRos::setup() {
    m_nh->subscribe(subTentacleOrders);
	m_nh->advertise(pubTentacleStates);
	m_nh->advertise(pubTentacleReleaseFbs);

    timerPubStates.reset();

    for (int i = 0; i < nbTentacles; i++) {
        tentacleStateMachineArray[i]->setTentacleRos(this);
    }
}

void TentacleRos::loop() {
    if (timerPubStates.startIfNotStartedAndTestExpiration(millis())) {
        for (int i = 0; i < nbTentacles; i++) {
            publishTentacleState(i);
        }
        timerPubStates.reset();
    }
}