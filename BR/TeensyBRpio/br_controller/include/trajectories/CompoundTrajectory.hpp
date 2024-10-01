#ifndef _COMPOUND_TRAJECTORY_HPP_
#define _COMPOUND_TRAJECTORY_HPP_

#include "trajectories/Trajectory.hpp"

#include <memory>

/// A trajectory consisting of the concatenation of two subtrajectories. (This is not used yet.)
class CompoundTrajectory : public Trajectory {
  public:
    /**
     * @param first,second At least one must be non-null! If they are both non-null, the trajectories should be contiguous.
     *
     * Note that if `first` or `second` are CompoundTrajectory themselves, the hierarchy is not flattened.
     */
    CompoundTrajectory(std::unique_ptr<Trajectory> first, std::unique_ptr<Trajectory> second);

    /// @copydoc Trajectory::advance()
    bool advance(double_t distance) override;

    /// @copydoc Trajectory::getCurrentPosition()
    Position2D<Meter> getCurrentPosition() const override;

    /// @copydoc Trajectory::getRemainingDistance()
    std::optional<double_t> getRemainingDistance() const override;

  private:
    // INVARIANT: m_first != null
    std::unique_ptr<Trajectory> m_first;
    std::unique_ptr<Trajectory> m_second;
};

#endif