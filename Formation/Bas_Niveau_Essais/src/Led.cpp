#include "Led.h"


Led::Led(int pin){//On cree le constructeur pour definir les variables internes
    m_pin = pin;
}

void Led::setup(){
    //PIN mode -> associe la pin à la var en inpput ou output
    pinMode(m_pin,OUTPUT) ;// On lui envoie l'info
    m_time = millis(); //On a aussi micros()
}


void Led::loop(){
    if (  millis() - m_time > 800 ){//prend la difference de temps car m_time a ete initialisé au setup (premiere fois)
        on();
        m_time = millis();
    } else if (millis() - m_time >600){
        off();
    }

}

void Led::on(){

    digitalWrite(m_pin, 1); // AnalogWrite(pin,valeur) : de 0 à 255 , digitalWrite : de 0 ou 1

}
void Led::off(){

    digitalWrite(m_pin, 0); // AnalogWrite(pin,valeur) : de 0 à 255 , digitalWrite : de 0 ou 1

}