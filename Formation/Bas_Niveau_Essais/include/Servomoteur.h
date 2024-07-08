#include <Arduino.h>
#include <ESP32Servo.h>
#ifndef SERVOMOTEUR_H
#define SERVOMOTEUR_H

//etat = machien à etat : utiliser enumerate

class Servomoteur{ //le nom de la classe correspond au nom du fichier

    private : 
        Servo servo_mot;
        long m_time;
        int m_pin;  //on ne va utiliser ces variables que dans le moteurs.cpp
    
    public :
    
        Servomoteur(int pin); //on ne met pas la var m_pin dans le cosntructeur
        void setup(); //on a des fonction
        void loop();
};


#endif