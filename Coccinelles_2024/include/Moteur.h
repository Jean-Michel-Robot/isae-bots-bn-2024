#include <Arduino.h>

#ifndef MOTEUR_H
#define MOTEUR_H

class Moteur{
    private:
        int m_EN;
        int m_IN1;
        int m_IN2;
        long m_vitesse;  
        // Facteur pour passer d'un entier entre 0 ET 255 à une vitesse en cm/s
        float K_conv ; 

    public: 
        void set_speed(int vitesse);
        
        void stop();

        Moteur(int EN,int IN1,int IN2);
        void setup();
        void loop();



};

#endif 