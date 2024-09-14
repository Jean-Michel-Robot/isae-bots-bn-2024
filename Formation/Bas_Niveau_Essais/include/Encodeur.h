
#ifndef ENCODEUR_H
#define ENCODEUR_H
#include <Arduino.h>
#include <ESP32Encoder.h>

class Encodeur{ //class moteur -> c'est l'objet
    public :

        void setup(); //methodes de classes
        void loop();
        Encodeur(int clk, int dt); //pin clk et dt, l'odre n'est pas grave, on peux les inverser sans souci
        void mesure_vitesse();


    private :
        ESP32Encoder encodeur;//Regarder la datasheet, on declare l'oibjet encodeur avec une fonction réalisée avant
        //peut etre utilisé avec les fonctions de la librairie ET dans le .cpp
        int m_clk; //On les declare ici et SEULEMENT ici
        int m_dt;
        long m_time;

        int m_ini;
        int m_fin;
        int m_t_ini;
        int m_t_fin;


};


#endif