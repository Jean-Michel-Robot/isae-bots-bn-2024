#ifndef _UNICYCLE_STATE_SIMULATOR_HPP_
#define _UNICYCLE_STATE_SIMULATOR_HPP_

#include "configuration.hpp"
#include "geometry/Position2D.hpp"
#include "geometry/Speeds.hpp"
#include "motors/MotorStub2.hpp"

#include <memory>
#include <optional>
#include <random>

/// See concept PositionFeedback.
class UnicycleStateSimulator {
  public:
    /**
     * @param noise_stddev Standard deviation for the simulation noise. Must be positive, or zero to disable the noise.
     * Currently, the noise follows a centered normal distribution and is added to both the linear and the angular speeds.
     * The standard deviation is the same for both speeds (which may not be physically homogeneous).
     * TODO: improve how the noise is implemented
     */
    UnicycleStateSimulator(double_t noise_stddev);
    UnicycleStateSimulator() : UnicycleStateSimulator(NOISE_STD_DEV) {}

    void setSpeeds(Speeds speeds);
    void resetPosition(Position2D<Millimeter> pos);
    void update(double_t interval);

    Position2D<Meter> getRobotPosition() const;
    MotorStub2 createMotorStub() const;

  private:
    class Noise {
      public:
        Noise(double_t standardDeviation);
        double_t operator()();

      private:
        std::optional<std::normal_distribution<double_t>> m_noise;
    };

    std::shared_ptr<Speeds> m_speeds;
    Position2D<Meter> m_position;
    Noise m_noise;
};

#endif