#include <Led.h>
#include <Servomoteur.h>
#include <Encodeur.h>
#include <Arduino.h>
#include <Moteur.h>

//Pas besoin de definir les pins de ground et + et -, c'est la teensy qui s'en charge
//on creer l'objet led
Led led = Led(2); //car on s'accroche au pin 2 grace à la datasheet

//création du servomoteur :
Servomoteur servomoteur = Servomoteur(22);
// COnstructeur nom = Constructeur(....)
Moteur moteurG = Moteur(12,13,27); //donné par l'elec
Moteur moteurD = Moteur(14,15,26);
Encodeur encodeur = Encodeur(5,18);

void setup(){
  //on appelle l'objet creer
  led.setup();
  servomoteur.setup();
  moteurG.setup();
  moteurD.setup();
  encodeur.setup();
  
  Serial.begin(115200);

}

void loop(){

  led.loop(); //pas de delai dans la carte, ce sont des actions idnependantes 
  encodeur.mesure_vitesse();
}


//TODO FAIRE la fct set_speed ou on peut reculer si vitesse négative