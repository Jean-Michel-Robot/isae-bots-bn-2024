#include "IsbHwHandler.h"

IsbHwHandler::IsbHwHandler(uint8_t data, uint8_t clk, uint8_t latch, uint8_t led)
    : m_isb(data, clk, latch, led)
{

    this->m_switchesState = 0;
    this->m_mactchStarted = false;
}

byte IsbHwHandler::getSwitchesState()
{
    return m_switchesState;
}

bool IsbHwHandler::isMatchStarted()
{
    return m_mactchStarted;
}

void IsbHwHandler::matchStarted()
{
    m_mactchStarted = true;
}

void IsbHwHandler::reset()
{
    m_mactchStarted = false;
}

void IsbHwHandler::readState()
{
    m_switchesState = m_isb.getByte();
}

void IsbHwHandler::setSinglePixel(uint8_t pixelPos, uint8_t r, uint8_t g, uint8_t b)
{
    m_isb.setSinglePixel(pixelPos, r, g, b);
    m_isb.show();
}

void IsbHwHandler::setup()
{
    m_isb.begin();
    m_isb.setBrightness(16);
    m_isb.clear();

    readState();
}

void IsbHwHandler::loop()
{
    static unsigned long lastTimeRefresh = 0;

    if (millis() - lastTimeRefresh > ISB_UPDATE_PERIOD)
    {
        lastTimeRefresh = millis();
        //m_isb.show();
        if (!m_mactchStarted) readState();
    }
}
