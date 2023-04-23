/*
   Permet la gestion de la LED RGB
*/
#ifndef __SIMU__
#else
#include "../Simulation/NeopixelSimu.h" // si en mode simu sur PC, on inclue la librairie simulée
#endif
// #include "a_Led_RGB.h"
// #include "a_define.h"

#include "LED.hpp"

LED::LED()
{
    m_pixels = new Adafruit_NeoPixel(2, 6, NEO_GRB); // 2 leds sur la pin 6
    m_pixels->begin(); // This initializes the NeoPixel library.
    // color(0,0,0);
    
}

void LED::color(int R, int G, int B)
{
  m_pixels->setPixelColor(0, m_pixels->Color(R, G, B));
  m_pixels->show(); // This sends the updated pixel color to the hardware.
}



BlinkLED::BlinkLED()
{
  pinMode(LED_BUILTIN, OUTPUT);
  m_state = 1;
  m_timer = 0.0;
}

void BlinkLED::loop()
{
  if(millis() - m_timer > 100){
    m_state = 1 - m_state;
    digitalWrite(13, m_state);
    m_timer = millis();
    // Serial.println("ting");
  }
}
