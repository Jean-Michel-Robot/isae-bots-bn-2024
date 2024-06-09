#include "Servomoteur.h"

Servomoteur::Servomoteur(int pin){
    m_pin = pin; //donne la variable de m_pin qui est var private
}

void Servomoteur::setup(){
    pinMode(m_pin,OUTPUT); //On a stocké la variable de m_pin dans le cosntructeur, oon y a accés dans ce fichier cpp seulement
    m_time = millis();
    servo_mot.attach(m_pin); //objet associé à la librairie

}

void Servomoteur::loop(){
    if (millis() - m_time > 4000){
        servo_mot.write(100); //servo_mot est un objet de librairie, il faut regarder comment le configurer
    } else if (millis() - m_time > 6000){
        servo_mot.write(20);
        m_time = millis();
    }
    
}