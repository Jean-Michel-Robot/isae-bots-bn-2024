#ifndef _FSM_STATE_MACHINE_HPP_
#define _FSM_STATE_MACHINE_HPP_

#include "defines/constraint.hpp"
#include "logging.hpp"

#include <memory>

namespace fsm {

/**
 * A state machine.
 *
 * Thse machine is created in an undefined state. It is undefined behaviour to call getCurrentState() before setCurrentState().
 * @tparam TState The base type of the machine's states. This can be a virtual type.
 */
template <typename TState>
class StateMachine {
  public:
    /**
     * Set the state of the machine. This destructs the current state and immediately invalidates any reference to it.
     * In particular, calling this method in a method of the current state is undefined behaviour.
     *
     * @tparam TNewState The type of the new state.
     * @param args The parameters to pass to the constructor of the new state.
     */
    template <Derived<TState> TNewState, typename... Args>
    void setCurrentState(Args &&...args) {
        m_currentState.reset();
        m_currentState = std::make_unique<TNewState>(std::forward<Args>(args)...);
    }

    /// Make sure to call setCurrentState() first!
    const TState &getCurrentState() const {
        checkHasState();
        return *m_currentState;
    }
    /// Make sure to call setCurrentState() first!
    TState &getCurrentState() {
        checkHasState();
        return *m_currentState;
    }

  private:
    std::unique_ptr<TState> m_currentState;

    void checkHasState() const {
        if (!m_currentState) {
            abort("StateMachine has null state! You are missing a call to setCurrentState(). Aborting...");
        }
    }
};
} // namespace fsm

#endif