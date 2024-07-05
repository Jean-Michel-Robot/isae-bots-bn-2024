/**
 * @file Serv.h
 * @brief Classe servant à controler un servo moteur
 * Pas utilisé pour la coupe 2024
 */

/**
 * Classe servo
 */
#ifndef SERV_H
#define SERV_H
#include <Arduino.h>
#include <ESP32Servo.h>

class Serv
{

private:
    Servo servo;
    int etat;
    int pin;
    long temps_servo;

public:
    Serv(int pin);
    void blink(long temps_blink, int angle1, int angle2);
    void setup();
    void loop();
};

#endif