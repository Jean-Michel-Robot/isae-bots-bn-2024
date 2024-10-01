#ifndef _MOTOR_STUB_HPP_
#define _MOTOR_STUB_HPP_

#include "geometry/Speeds.hpp"

#include <memory>

/**
 * Motor stub for a unicycle robot. This class sends the requested command to its consumer (usually UnicycleStateSimulator) and does
 * nothing else.
 */
class MotorStub2 {
  public:
    /// Use UnicycleStateSimulator::createMotorStub() to create a motor stub connected to the simulator.
    /// @param speedsPtr must not be null.
    MotorStub2(std::shared_ptr<Speeds> speedsPtr);

    // See Actuators.hpp

    void switchOn();
    void switchOff();
    void update(double_t interval);
    bool isReady() const;
    bool isIdle() const;
    void sendCommand(Speeds speeds);

  private:
    std::shared_ptr<Speeds> m_speeds;
    bool m_isReady;
};

#endif