#include <ESP32Encoder.h>
#include <Arduino.h>

#ifndef ENCODEUR_H
#define ENCODEUR_H

class Encodeur{
    private: 
       ESP32Encoder encoder ; 
       int m_clk ; 
       int m_dt ;
       long m_time ; 
    public: 
        Encodeur(int clk , int dt );
        void setup();
        void loop();
        long mesure();
        
       
};

#endif
