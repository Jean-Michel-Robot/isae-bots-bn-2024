#include "Moteur.h"

Moteur::Moteur(int in1, int in2, int en){ //Moteur : classe et Moteur::Moteur constructeur
    m_in1 = in1;
    m_in2 = in2;
    m_en = en;
};

void Moteur::setup(){
    pinMode(m_in1, OUTPUT); //On est dans le moteur donc on peux utiliser m_pin 
    pinMode(m_in2, OUTPUT);
    pinMode(m_en, OUTPUT);
};

void Moteur::loop(){

};

void Moteur::set_speed(int vitesse){ 

    int v;
    if (vitesse > 255){
        v = 255;
    } else if (vitesse < -255){
        v = -255;
    } else{
        v = vitesse;
    }

    if (v > 0){
        //Moteur en position avant
        digitalWrite(m_in1, 1); // Inverser ou non le sens du courant
        digitalWrite(m_in2, 0);
        analogWrite(m_en, v);
    } else if  (v < 0){
        //Moteur en position arrière
        digitalWrite(m_in1, 0);
        digitalWrite(m_in2, 1);
        
        v = -v;
        //Serial.println(v); Permet d'afficher une valeur lors du debuggage
        analogWrite(m_en, v);
    } else {
        analogWrite(m_en, 0);
    }

};

