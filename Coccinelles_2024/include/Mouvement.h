#include <Arduino.h>
#include <Mesure_pos.h>
#ifndef MOUVEMENT_H
#define MOUVEMENT_H

class Mouvement{
    private:
    Mesure_pos* m_p_mesure_pos;
    public:
    Mouvement(Mesure_pos* p_mesure_pos);
    void setup();
    void loop();
    void avancer(float vitesse_x, float vitesse_y);
    void tourner(float angle);
    void stop();
};
#endif