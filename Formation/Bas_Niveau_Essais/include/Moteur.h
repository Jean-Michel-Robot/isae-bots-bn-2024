#include <Arduino.h>
#ifndef MOTEURS_H
#define MOTEURS_H

//un moteur a besoin de input 1 et 2, il a besoin d'un controlleur
//enable 1 et 2, c'est pour relier la L239, le controlleur à l'ESP "32", mais aussi pour alimenter les moteurs en puissance (activer PWM)
//On ne branche pas le moteur au 3v3 de l'esp

class Moteur{ //class moteur -> c'est l'objet
    public :

        void setup(); //methodes de classes
        void loop();
        Moteur(int in1, int in2, int en);
        void set_speed(int vitesse); // De 0 à 255

    private :

        int m_in1; //avec m, on a les variables privées,(c'est un attribut = argument de la classe) sinon c'est publique
        int m_in2;
        int m_en;


};


#endif