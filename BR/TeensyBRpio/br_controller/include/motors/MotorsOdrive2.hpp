#ifndef _MOTORS_ODRIVE_2_HPP_
#define _MOTORS_ODRIVE_2_HPP_

#include "configuration.hpp"
#include "defines/constraint.hpp"
#include "geometry/Speeds.hpp"

#include <ODriveArduino.h>
#include <SoftwareSerial.h>
#include <memory>

/**
 * Interface with two differential Odrive motors. Arduino only.
 * Satisfies concept Actuators.
 */
class MotorsOdrive2 {
  public:
    MotorsOdrive2(uint8_t odriveRxPin, uint8_t odriveTxPin, double_t transmissionRatio, double_t wheelDiameter, double_t wheelDistance,
                  double_t maxMotorSpeed);

    MotorsOdrive2() : MotorsOdrive2(ODRIVE_RX_PIN, ODRIVE_TX_PIN, TRANSMISSION_RATIO, WHEEL_DIAMETER, WHEEL_DISTANCE, MAX_MOTOR_SPEED) {}

    // See Actuators.hpp

    void switchOn();
    void switchOff();
    void update(double_t interval);
    bool isReady();
    bool isIdle();
    void sendCommand(Speeds speeds);

#ifdef _DEBUG
    double_t getLastLeftSpeed() const;
    double_t getLastRightSpeed() const;
#endif

  private:
    void sendCommand(int motor_number, double_t velCmd);

#ifdef _DEBUG
    double_t m_lastLeftSpeed;
    double_t m_lastRightSpeed;
#endif

    std::unique_ptr<SoftwareSerial> m_serial;
    ODriveArduino m_odrive;

    double_t m_transmissionRatio;
    double_t m_wheelDiameter;
    double_t m_wheelDistance;
    double_t m_maxMotorSpeed;
};

#endif