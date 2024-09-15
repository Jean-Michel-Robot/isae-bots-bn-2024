#ifndef _H_SOFT_ODO_ENCODER
#define _H_SOFT_ODO_ENCODER

// TODO: this seems to be an old library that was already archived and removed from the project before the refactoring.
#include "Encoder/Encoder.h"
#include "OdoEncoder.hpp"

class SoftOdoEncoder : public OdoEncoder
{
public:
    SoftOdoEncoder(): knobLeft(32, 25), knobRight(3, 4) { }

    void resetCounters() override
    {
        knobLeft.write(0);
        knobRight.write(0);
    }
    void getLeftCounter() override
    {
        return -knobLeft.read();
    }
    void getRightCounter() override
    {
        return knobRight.read();
    }

    private:
    Encoder knobLeft;
    Encoder knobRight;
}

#endif