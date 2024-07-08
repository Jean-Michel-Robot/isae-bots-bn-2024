#include <Arduino.h>
#include <Servo.h>

Servo servo;
void moveServo()
{
  static int angle = 0; // initial angle
  servo.attach(4);
  servo.write(angle); // move servo to current angle
  angle += 90;        // increment angle by 90 degrees
  if (angle > 180)
  {
    angle = 0; // reset angle to 0 if it exceeds 180 degrees
  }
  servo.detach(); // release servo
}

void setup()
{
  // put your setup code here, to run once:
  servo.attach(4); // attach servo to pin 21
}

void loop()
{
  // put your main code here, to run repeatedly:0
  servo.write(40); // move servo to 0 degrees
  delay(1000);
  servo.write(5);
  delay(1000); // delay for 1 second
}
// put function definitions here: