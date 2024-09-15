#ifndef _H_LOGGING
#define _H_LOGGING

enum LogType
{
    INFO = 0,
    WARN = 1,
    ERROR = 2,
    FATAL = 3,
    DEBUG = 4,
};

#ifdef ARDUINO
using string_t = String;
#else
using string_t = const char*;
#endif

// The caller does not have to know how the message is actually logged.
// This will (but does not have to) delegate logging to ROS2
// The state machine part should not have a dependency on ROS.
void log(const LogType type, string_t message);

// Defined in ROS_*.cpp

#endif