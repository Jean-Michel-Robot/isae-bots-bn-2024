#include <Arduino.h>
#include <Servo.h>

Servo servo;
void moveServo()
{
  static int angle = 0; // initial angle
  servo.attach(2);
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
  servo.attach(/* servo pin number */);
}

void loop()
{
  // put your main code here, to run repeatedly:
  moveServo();
  delay(1000); // delay for 1 second
}

// put function definitions here:
int myFunction(int x, int y)
{
  return x + y;
}