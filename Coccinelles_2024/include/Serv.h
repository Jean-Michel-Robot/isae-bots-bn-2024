#include <Arduino.h>
#include <ESP32Servo.h>


/**
 * Classe servo 
*/
#ifndef SERV_H
#define SERV_H


class Serv{

private: 
    Servo servo ; 
    int etat; 
    int pin ; 
    long temps_servo ; 



public : 
   Serv(int pin );
   void blink(long temps_blink, int angle1 , int angle2 );
   void setup();
   void loop();
};

#endif 