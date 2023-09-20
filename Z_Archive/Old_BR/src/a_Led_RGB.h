#ifndef __H_LED_RGB
#define __H_LED_RGB
class Adafruit_NeoPixel;// forward déclaration : on indique au compilateur que la classe sera définie plus tard
// cela suffit pour faire des pointeurs, et ça évite d'inclure inutilement des headers dans les headers
// ideal pour les libs arduino qui ont des headers assez fournis

#include "a_define.h"

class Led
{
public :
    Led(); // constructeur sans type
    // pas de destructeur car pas de situation où on "ferme" le programme (le code s'arrête brutalement à la coupure du jus)
    void color(int R, int G, int B);
protected :
    Adafruit_NeoPixel *m_pixels;
};
#endif
