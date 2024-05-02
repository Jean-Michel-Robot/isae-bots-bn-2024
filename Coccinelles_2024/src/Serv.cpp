
#include <Serv.h>

Serv::Serv(int pin)
{
  this->servo = Servo();
  this->pin = pin;
}

void Serv::blink(long temps_blink, int angle1, int angle2)
{
  if (millis() - temps_servo > temps_blink)
  {
    if (etat == angle1)
    {
      this->servo.write(angle2);
      etat = angle2;
    }
    else
    {
      this->servo.write(angle1);
      etat = angle1;
    }
    temps_servo = millis();
  }
}

void Serv::setup()
{
  temps_servo = millis();
  servo.attach(pin);

  etat = 120;
  servo.write(180);
}

void Serv::loop()
{
  servo.write(150);
  delay(1000);
  servo.write(80);
}
