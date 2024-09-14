
#include <Encodeur.h>

Encodeur::Encodeur(int clk, int dt){
    m_clk = clk;
    m_dt = dt;
};

void Encodeur::setup(){
    encodeur.attachHalfQuad(m_clk, m_dt);
    encodeur.setCount(0);
    m_time = millis();
    m_ini = 0;
    m_fin = 0;
    m_t_ini = 0;
    m_t_fin = 0;
};

void Encodeur::loop(){
    Serial.println("Le nombre de tics : "+ String(encodeur.getCount()));

};

void Encodeur::mesure_vitesse(){
    // & tour de roue = 1945 tics

    float vitesse;
    if (millis() - m_time < 1){
        m_ini = encodeur.getCount();
        m_t_ini = millis();
    } else if (millis() - m_time > 800){
        m_fin = encodeur.getCount();
        m_t_fin = millis();

        vitesse = (m_fin-m_ini)/(m_t_fin-m_t_ini); //tics par secondes
        vitesse = (vitesse/1945.0)*(64.2)*3.1415;

        Serial.println(vitesse);
        
    }
    


};

