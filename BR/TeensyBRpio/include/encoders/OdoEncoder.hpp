#ifndef _H_ODO_ENCODER
#define _H_ODO_ENCODER

#include <stdint.h>

class OdoEncoder {
    public:
    virtual void resetCounters() = 0;
    virtual int32_t getLeftCounter() const = 0;
    virtual int32_t getRightCounter() const = 0;
};

#endif