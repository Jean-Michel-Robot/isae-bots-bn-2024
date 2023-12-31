/*
   Permet la gestion de la LED RGB
*/
#ifndef __SIMU__
#include "src/Adafruit_NeoPixel/Adafruit_NeoPixel.h"
#else
#include "../Simulation/NeopixelSimu.h" // si en mode simu sur PC, on inclue la librairie simulée
#endif
#include "a_Led_RGB.h"
#include "a_define.h"

Led::Led()
{
    m_pixels = new Adafruit_NeoPixel(1, 6, NEO_GRB + NEO_KHZ800); //1 led sur le pin 6
    m_pixels->begin(); // This initializes the NeoPixel library.
    color(0,0,0);
}
void Led::color(int R, int G, int B)
{
  m_pixels->setPixelColor(0, m_pixels->Color(R, G, B));
  m_pixels->show(); // This sends the updated pixel color to the hardware.
}

