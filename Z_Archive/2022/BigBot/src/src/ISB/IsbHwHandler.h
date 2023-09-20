#ifndef ISBHWHANDLER_H
#define ISBHWHANDLER_H

#include <Arduino.h>
#include "ISB.h"

#define ISB_UPDATE_PERIOD 500 // ms

class IsbHwHandler
{
private:
    ISB m_isb; // ISB associated with this handler
    byte m_switchesState; // switches state
    bool m_mactchStarted;

    void readState();

public:
    IsbHwHandler(uint8_t data, uint8_t clk, uint8_t latch, uint8_t led); // constructor of IsbHandler class

    byte getSwitchesState();

    bool isMatchStarted();

    void matchStarted();

    void reset();

    void setSinglePixel(uint8_t pixelPos, uint8_t r, uint8_t g, uint8_t b);

    void setup();

    void loop();
};

#endif
