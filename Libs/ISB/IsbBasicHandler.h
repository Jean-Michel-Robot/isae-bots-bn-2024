/*
    Allows for the creation of IsbBasicHandler objects that will handle basic config and test that apply to both robots
    (tirette, side color, chosen strat)
*/

#ifndef ISBBASICHANDLER_H
#define ISBBASICHANDLER_H

#include <Arduino.h>
#include "IsbHwHandler.h"

#define NB_PUB_START 10

class IsbBasicRos;

class IsbBasicHandler
{
public:
    enum TiretteState
    {
        TIRETTE_IN = 0,
        TIRETTE_OUT = 1
    };

    IsbBasicHandler(IsbHwHandler *isbHwHandler,
        uint8_t posTirette, uint8_t posSide, uint8_t posStrat,
        int nbStrat); // constructor of IsbBasicHandler class

    IsbHwHandler *getIsbHwHandler();

    bool getSide();

    void setIsbBasicRos(IsbBasicRos *isbBasicRos);

    void handleStateChanges();

    void reset();

    void setup();

    void loop();

private:
    IsbHwHandler *m_isbHwHandler; // isb hardware handler

    uint8_t m_posTirette; // position of the tirette
    uint8_t m_posSide; // position of side switch
    uint8_t m_posStrat; // position of strat switch

    uint8_t m_maskTirette; // mask that gives the tirette switch position
    uint8_t m_maskSide; // mask that gives the side choosing switch position
    uint8_t m_maskStrat; // mask that gives the strat choosing switch position

    TiretteState m_currentTiretteState; // tirette state (in, out)
    bool m_side; // side color
    uint8_t m_stratSwitch; // state of the strat switch

    int m_nbStrat; // total number of strategies
    int m_currentStrat; // strat id

    // Ros communication
    IsbBasicRos *m_isbBasicRos;
    int m_countPubMatchStarted = 0;

};

#endif