#ifndef _DEFINE_LOGGING_HPP_
#define _DEFINE_LOGGING_HPP_

#include "defines/string.h"

#ifndef __EXCEPTIONS
#include <cstdlib>
#endif

enum LogSeverity {
    DEBUG,
    INFO,
    WARN,
    ERROR,
    FATAL,
};

inline string_t severityToString(LogSeverity severity) {
    switch (severity) {
        case DEBUG:
            return "DEBUG";
        case INFO:
            return "INFO";
        case WARN:
            return "WARN";
        case ERROR:
            return "ERROR";
        case FATAL:
            return "FATAL";
        default:
            return "INVALID";
    }
}

void log(LogSeverity severity, string_t message);

[[noreturn]] inline void abort(string_t message) {
#ifdef __EXCEPTIONS
    throw message;
#else
    log(FATAL, message);
    std::abort();
#endif
}

#endif