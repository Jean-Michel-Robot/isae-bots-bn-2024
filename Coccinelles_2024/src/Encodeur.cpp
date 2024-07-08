#include <Encodeur.h>

Encodeur::Encodeur(int clk , int dt ){
    m_clk = clk ; 
    m_dt = dt ; 
}

void Encodeur::setup(){
    m_time = millis();
    encoder.attachHalfQuad(m_dt,m_clk);
    encoder.setCount(0);
}

long Encodeur::mesure(){
    return encoder.getCount();

}

void Encodeur::loop(){
    if (millis()-m_time>1000){
        Serial.println(mesure()); // affiche la valeur du compteur de l'encodeur
        m_time = millis();
    }
}