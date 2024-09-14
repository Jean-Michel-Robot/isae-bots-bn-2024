#include <Moteur.h>

Moteur::Moteur(int EN, int IN1, int IN2)
{
    m_EN = EN;
    m_IN1 = IN1;
    m_IN2 = IN2;
}

void Moteur::setup()
{
    pinMode(m_EN, OUTPUT);
    pinMode(m_IN1, OUTPUT);
    pinMode(m_IN2, OUTPUT);
    digitalWrite(m_IN1, 0);
    digitalWrite(m_IN2, 1);
}

void Moteur::set_speed(int vitesse)
{
    if (vitesse < 0)
    {
        digitalWrite(m_IN1, 1);
        digitalWrite(m_IN2, 0);
        m_vitesse = -vitesse;
    }
    else
    {
        digitalWrite(m_IN1, 0);
        digitalWrite(m_IN2, 1);
        m_vitesse = vitesse;
    }
    if (m_vitesse > 255)
    {
        m_vitesse = 255;
    }
    analogWrite(m_EN, m_vitesse);
}

void Moteur::stop(){
    digitalWrite(m_IN1, 0);
    digitalWrite(m_IN2, 0);
    analogWrite(m_EN, 0);
}

void Moteur::loop()
{
}

