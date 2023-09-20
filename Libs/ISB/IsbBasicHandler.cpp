/*
    Allows for the creation of IsbBasicHandler objects that will handle basic config and test that apply to both robots
    (tirette, side color, chosen strat)
*/

#include "IsbBasicHandler.h"
#include "IsbBasicRos.h"

// constructor of IsbBasicHandler class
IsbBasicHandler::IsbBasicHandler(IsbHwHandler *isbHwHandler,
    uint8_t posTirette, uint8_t posSide, uint8_t posStrat,
    int nbStrat)
{
    this->m_isbHwHandler = isbHwHandler;

    this->m_posTirette = posTirette;
    this->m_posSide = posSide;
    this->m_posStrat = posStrat;

    this->m_maskTirette = (1 << m_posTirette);
    this->m_maskSide = (1 << m_posSide);
    this->m_maskStrat = (255 << m_posStrat);

    this->m_currentTiretteState = TIRETTE_OUT;
    this->m_side = 0;
    this->m_stratSwitch = 0;

    this->m_nbStrat = nbStrat;
    this->m_currentStrat = 0;

    this->m_isbBasicRos = NULL;
    this->m_countPubMatchStarted = 0;

}

IsbHwHandler *IsbBasicHandler::getIsbHwHandler()
{
    return m_isbHwHandler;
}

bool IsbBasicHandler::getSide()
{
    return m_side;
}

void IsbBasicHandler::setIsbBasicRos(IsbBasicRos *isbBasicRos)
{
    this->m_isbBasicRos = isbBasicRos;
}

void IsbBasicHandler::handleStateChanges()
{
    
    // store old variables
    TiretteState oldTiretteState = m_currentTiretteState;
    bool oldSide = m_side;
    int oldStratSwitch = m_stratSwitch;

    // update variables with new value
    m_currentTiretteState = (TiretteState) ((m_isbHwHandler->getSwitchesState() & m_maskTirette) >> m_posTirette);
    m_side = (m_isbHwHandler->getSwitchesState() & m_maskSide) >> m_posSide;
    m_stratSwitch = (m_isbHwHandler->getSwitchesState() & m_maskStrat) >> m_posStrat;

    //Serial.println(m_isbHwHandler->getSwitchesState(), BIN);

    if (m_side != oldSide)
    {
        //Serial.println("New side: " + String(m_side));
        if (m_isbBasicRos != NULL) m_isbBasicRos->publishIsbSide(m_side);
    }

    if (m_stratSwitch != oldStratSwitch)
    {
        m_currentStrat = m_stratSwitch;
        //Serial.println("New strat: " + String(m_currentStrat));
        if (m_isbBasicRos != NULL) m_isbBasicRos->publishIsbStrat(m_currentStrat);
    }

    if (oldTiretteState == TIRETTE_IN && m_currentTiretteState == TIRETTE_OUT)
    {
        m_isbHwHandler->matchStarted();
        //Serial.println("Match started!");
    }
    
}

void IsbBasicHandler::reset()
{
    m_countPubMatchStarted = 0;
}

void IsbBasicHandler::setup()
{
    // update variables with first value
    m_currentTiretteState = (TiretteState) ((m_isbHwHandler->getSwitchesState() & m_maskTirette) >> m_posTirette);
    m_side = (m_isbHwHandler->getSwitchesState() & m_maskSide) >> m_posSide;
    m_stratSwitch = (m_isbHwHandler->getSwitchesState() & m_maskStrat) >> m_posStrat;
}

void IsbBasicHandler::loop()
{
    if (!m_isbHwHandler->isMatchStarted())
    {
        handleStateChanges();
        return;
    }

    if (m_countPubMatchStarted < NB_PUB_START)
    {
        if (m_isbBasicRos != NULL) m_isbBasicRos->publishIsbMatchStarted();
        m_countPubMatchStarted++;
    }
}

