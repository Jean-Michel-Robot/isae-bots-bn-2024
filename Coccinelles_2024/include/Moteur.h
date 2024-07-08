/**
 * @file Moteur.h
 * @brief Classe pour controler un moteur
 *
 */

#include <Arduino.h>

#ifndef MOTEUR_H
#define MOTEUR_H

class Moteur
{
private:
    int m_EN;  // pin de la pwm
    int m_IN1; // pin de direction 1
    int m_IN2; // pin de direction 2
    long m_vitesse;
    // Facteur pour passer d'un entier entre 0 ET 255 à une vitesse en cm/s
    float K_conv;

public:
    /**
     * @brief met la vitesse du moteur à vitesse
     *
     * @param vitesse : speed of the motor is a int between 0 and 255
     */
    void set_speed(int vitesse);
    /**
     * @brief Pas utilisé , arrête le moteur
     */
    void stop();

    /**
     * @brief constructeur
     */
    Moteur(int EN, int IN1, int IN2);
    /**
     * @brief Initialisation du moteur
     */
    void setup();
    void loop();
};

#endif