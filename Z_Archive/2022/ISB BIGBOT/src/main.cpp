#include "Arduino.h"

#include "clsPCA9555.h"
#include "Wire.h"

PCA9555 ioport(0x00);

void setup()
{
	// start I2C
	ioport.begin();

	// sets the I2C clock to 400kHz
	ioport.setClock(400000);

	// set first 14 pins to input
	ioport.pinMode(P00, INPUT);
  ioport.pinMode(P01, INPUT);
  ioport.pinMode(P02, INPUT);
  ioport.pinMode(P03, INPUT);
  ioport.pinMode(P04, INPUT);
  ioport.pinMode(P05, INPUT);

}

void loop()
{
	int read = ioport.digitalRead(P01);
  Serial.println(read);
}