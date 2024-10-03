#ifdef ARDUINO

#include "motors/MotorsOdrive2.hpp"
#include <ODriveEnums.h>

#include <algorithm>
#include <numbers>

#include "logging.hpp"

// defines to avoid wondering if 0 is left or right
#define BR_RIGHT 0
#define BR_LEFT 1

MotorsOdrive2::MotorsOdrive2(uint8_t odriveRxPin, uint8_t odriveTxPin, double_t transmissionRatio, double_t wheelDiameter, double_t wheelDistance,
                             double_t maxMotorSpeed)
    : m_serial(std::make_unique<SoftwareSerial>(odriveRxPin, odriveTxPin)), m_odrive(*m_serial), m_transmissionRatio(transmissionRatio),
     m_wheelDiameter(wheelDiameter), m_wheelDistance(wheelDistance), m_maxMotorSpeed(maxMotorSpeed) {
    m_serial->begin(115200);
}

void MotorsOdrive2::sendCommand(Speeds speeds) {

    double_t rightWheelSpeed = speeds.linear + speeds.angular * m_wheelDistance / 2;
    double_t leftWheelSpeed = speeds.linear - speeds.angular * m_wheelDistance / 2;

    // transform velCmd into odrive command (nb_turn/s)
    //  knowing the wheel diameter and the transmission ratio
    double_t conversionFactor = m_transmissionRatio / (std::numbers::pi_v<double_t> * m_wheelDistance);

#ifdef _DEBUG
    m_lastLeftSpeed = -leftWheelSpeed * conversionFactor;
    m_lastRightSpeed = rightWheelSpeed * conversionFactor;
#endif

    sendCommand(BR_LEFT, -leftWheelSpeed * conversionFactor);
    sendCommand(BR_RIGHT, rightWheelSpeed * conversionFactor);
}

void MotorsOdrive2::sendCommand(int motor_number, double_t velCmd) {
    // constrain the motor command for safety
    m_odrive.SetVelocity(motor_number, std::clamp(velCmd, -m_maxMotorSpeed, m_maxMotorSpeed));
}

void MotorsOdrive2::switchOn() {
    // NOTE on n'attend pas le retour de l'Odrive
    m_odrive.run_state(BR_RIGHT, AXIS_STATE_CLOSED_LOOP_CONTROL, false, 0.0);
    m_odrive.run_state(BR_LEFT, AXIS_STATE_CLOSED_LOOP_CONTROL, false, 0.0);
}

void MotorsOdrive2::switchOff() {
    m_odrive.run_state(BR_RIGHT, AXIS_STATE_IDLE, false, 0.0);
    m_odrive.run_state(BR_LEFT, AXIS_STATE_IDLE, false, 0.0);
}

void MotorsOdrive2::update(double_t interval) {
    // Nothing to do
}

bool MotorsOdrive2::isReady() {
    return true;
    return m_odrive.getCurrentAxisState(BR_LEFT) == AXIS_STATE_CLOSED_LOOP_CONTROL &&
           m_odrive.getCurrentAxisState(BR_RIGHT) == AXIS_STATE_CLOSED_LOOP_CONTROL;
}
bool MotorsOdrive2::isIdle() {
    return m_odrive.getCurrentAxisState(BR_LEFT) == AXIS_STATE_IDLE && m_odrive.getCurrentAxisState(BR_RIGHT) == AXIS_STATE_IDLE;
}

#ifdef _DEBUG
double_t MotorsOdrive2::getLastLeftSpeed() const {
    return m_lastLeftSpeed;
}
double_t MotorsOdrive2::getLastRightSpeed() const {
    return m_lastRightSpeed;
}
#endif

#endif