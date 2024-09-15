#ifndef _H_HARD_ODO_ENCODER
#define _H_HARD_ODO_ENCODER

#include "QuadDecode.h"
#include "OdoEncoder.hpp"

class HardOdoEncoder : public OdoEncoder
{
public:
    HardOdoEncoder() = default;

    void resetCounters() override
    {
        QuadDecode::instance().resetCounter1();
        QuadDecode::instance().resetCounter2();
    }
    int32_t getLeftCounter() const override
    {
        return -QuadDecode::instance().getCounter2();
    }
    int32_t getRightCounter() const override
    {
        return QuadDecode::instance().getCounter1();
    }

    static HardOdoEncoder &instance() {
        static HardOdoEncoder encoder;
        return encoder;
    }
};

#endif