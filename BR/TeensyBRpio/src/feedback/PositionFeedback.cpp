#include "feedback/PositionFeedback.hpp"
#include "utils/clock.h"
#include "ros/ROS.hpp"
#include "defines.hpp"

void PositionFeedback::loop() {
    update();
    if (millis() - m_timer_last_send > ODO_SEND_POSITION_TIMER) {
        m_timer_last_send = millis();
        sendPosition();
    }
}

void PositionFeedback::resetPosition(Position2D<Millimeter> pos) {
    m_position = convert(pos);
}

Position2D<Meter> PositionFeedback::getRobotPosition() const {
    return m_position;
}

void PositionFeedback::sendPosition() const {
    ROS::instance().sendCurrentPosition(convert(m_position));
}