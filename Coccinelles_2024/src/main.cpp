
#include <Moteur.h>
#include <Arduino.h>
#include <ESP32Encoder.h>
#include <Encodeur.h>
#include <Mesure_pos.h>
#include <Serv.h>
#include <define.h>
#include <Asserv.h>
#include <Irsensor.h>
#include <Machine_etats.h>

#define IR_PIN 4
//  ne sert à rien pour le moment

Irsensor irsensor = Irsensor(IR_PIN);
Moteur moteur_d = Moteur(EN_R, IN1_R, IN2_R);
Moteur moteur_g = Moteur(EN_L, IN1_L, IN2_L);
Encodeur encoder_R = Encodeur(CLK_R, DT_R);
Encodeur encoder_L = Encodeur(CLK_L, DT_L);
Mesure_pos mesure_pos = Mesure_pos(&encoder_R, &encoder_L);
Asserv asserv = Asserv(&moteur_d, &moteur_g, &mesure_pos);

Machine_etats machine_etats = Machine_etats(&asserv, &mesure_pos, &irsensor);

void setup()
{
  irsensor.setup();
  mesure_pos.setup();
  //     encoder_L.setup();
  //      encoder_R.setup();
  Serial.begin(115200);
  moteur_g.setup();
  moteur_d.setup();

  // moteur_d.set_speed(175);
  // moteur_g.set_speed(175);
  // asserv.setup();
  // machine_etats.setup();

  // put your setup code here, to run once:
}

void loop()
{
  // encoder_L.loop();
  // encoder_R.loop();
  irsensor.loop();
  // mesure_pos.loop();
  //  asserv.loop();
  // machine_etats.loop();
  //       moteur_g.loop();
  //       moteur_d.loop();
}
