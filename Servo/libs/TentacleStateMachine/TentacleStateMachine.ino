#include "TentacleStateMachine.h"

#define SWITCH_PIN 15
#define SWITCH_TAU 0.05
#define SWITCH_THRESHOLD 0.5

#define PUMP_PIN 29

#define SERVO_PIN 20

String id = "TAVD";

String servoId = "SAVD";

int servoPositions[] = {135, 55, 45, 40};

#define TIME_IN_CONTACT 0.1 // s
#define TIME_LOST_CONTACT 0.5 // s

#define TIME_GRABBING 0.8 // s

#define TIME_LOST_CONTACT_BEFORE_PUMP_OFF_RELEASING 0.1 // s

#define TIME_BEFORE_PUMP_OFF_RELEASING_FROM_MID 0.3 // s
#define TIME_RELEASING_FROM_MID 2.0 // s

#define TIME_BEFORE_PUMP_OFF_RELEASING_FROM_HIGH 0.8 // s
#define TIME_RELEASING_FROM_HIGH 2.5 // s

TentacleStateMachine tentacleStateMachine = TentacleStateMachine(id, PUMP_PIN,
    SWITCH_PIN, SWITCH_TAU, SWITCH_THRESHOLD,
    servoId, SERVO_PIN, servoPositions,
    TIME_IN_CONTACT, TIME_LOST_CONTACT, TIME_GRABBING,
    TIME_LOST_CONTACT_BEFORE_PUMP_OFF_RELEASING,
    TIME_BEFORE_PUMP_OFF_RELEASING_FROM_MID, TIME_RELEASING_FROM_MID,
    TIME_BEFORE_PUMP_OFF_RELEASING_FROM_HIGH, TIME_RELEASING_FROM_HIGH);

Timer timerTimeoutSerial = Timer(5);
Timer timerPlot = Timer(0.1);
Timer timerSetup = Timer(3);
Timer timerCarrying = Timer(3);
Timer timerRestart = Timer(5);

bool firstTryReleasing = true;

void setup() {
    Serial.begin(9600);
    //while(!Serial);
    while(!(Serial || timerTimeoutSerial.startIfNotStartedAndTestExpiration(millis())));
    timerTimeoutSerial.reset();

    Serial.print("currentState");
    Serial.print(",");
    Serial.print("tentacleCurrentPos");
    Serial.print(",");
    Serial.println("NONE");

    tentacleStateMachine.setup();
    while (!timerSetup.startIfNotStartedAndTestExpiration(millis())) {
        if (timerPlot.startIfNotStartedAndTestExpiration(millis())) {
            Serial.print(tentacleStateMachine.getCurrentState());
            Serial.print(",");
            Serial.println(tentacleStateMachine.getTentacleCurrentPos());
            timerPlot.reset();
        }
    }
    timerSetup.reset();
    tentacleStateMachine.stateDoTransition(STATE_GRABBING);
}

void loop() {
    tentacleStateMachine.loop();

    if (timerPlot.startIfNotStartedAndTestExpiration(millis())) {
        Serial.print(tentacleStateMachine.getCurrentState());
        Serial.print(",");
        Serial.println(tentacleStateMachine.getTentacleCurrentPos());
        timerPlot.reset();
    }

    if (firstTryReleasing && tentacleStateMachine.getCurrentState() == STATE_CARRYING && timerCarrying.startIfNotStartedAndTestExpiration(millis())) {
      firstTryReleasing = false;
      tentacleStateMachine.stateDoTransition(STATE_RELEASING);
      timerCarrying.reset();
    }

    if (!firstTryReleasing && tentacleStateMachine.getCurrentState() == STATE_CARRYING_HIGH && timerCarrying.startIfNotStartedAndTestExpiration(millis())) {
      tentacleStateMachine.stateDoTransition(STATE_RELEASING);
      timerCarrying.reset();
    }

    if (tentacleStateMachine.getCurrentState() == STATE_IDLE && timerRestart.startIfNotStartedAndTestExpiration(millis())) {
      firstTryReleasing = true;
      Serial.print("currentState");
      Serial.print(",");
      Serial.print("tentacleCurrentPos");
      Serial.print(",");
      Serial.println("NONE");
      tentacleStateMachine.stateDoTransition(STATE_GRABBING);
      timerRestart.reset();
    }
}
