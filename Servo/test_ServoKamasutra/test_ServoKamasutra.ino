#include "ServoKamasutra.h"

#define SERVO_0_PIN 4
#define BUTTON_PIN 6

String id = "SERVO";
int nbPos0 = 3;
int positions0[] = {20, 100, 90};

ServoKamasutra servo0 = ServoKamasutra(id, SERVO_0_PIN, nbPos0, positions0);

void setup() {
  Serial.begin(9600);

  pinMode(13, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  
  servo0.setup();
  delay(2000);
}

void loop() {
//  Serial.println(positions0[2]);
//  servo0.setPosition(0);
//  delay(1000);
//  servo0.setPosition(1);
//  delay(1000);
//  servo0.setPosition(2);
//  delay(1000);

//  positions0[2] = 0;//changes to the positions array outside the object do not affect it

  if (digitalRead(BUTTON_PIN) == HIGH) {
    servo0.setPosition(0);
  }
  else {
    servo0.setPosition(1);
  }
}
