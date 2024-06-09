#include <Arduino.h>

#ifndef LED_H
#define LED_H

class Led{ //le nom de la classe correspond au nom du fichier + maj sur le nom de la classe

    public :
        int m_pin;
        void on();
        void off();
        void setup();
        void loop();

        Led(int pin);

    private:
        long m_time;

};

#endif
