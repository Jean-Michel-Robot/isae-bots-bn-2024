#ifndef __ISB_H__
#define __ISB_H__

//#define _DEBUG_

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// Definition of tempos for the clock register (micros)
#define CLOCK_TEMPO 1000
#define LATCH_TEMPO 1000

// Definition of tempos for others
#define TEMPO_CHEN 100

// Default board
#define NB_PIXELS 8
#define NB_SWITCH 8

// Colors
typedef enum e_Colorled
{
	RED,
	GREEN,
	BLUE,
	ORANGE,
	PURPLE,
	YELLOW,
	WHITE,
	NONE = -1
} Colorled;

class ISB
{
public:
	ISB(uint8_t data, uint8_t clk, uint8_t latch, uint8_t led);
	ISB(uint8_t data, uint8_t clk, uint8_t latch, uint8_t led, int m_nbSwitch, int m_nbPixels);
	~ISB(void);

	int begin(void);

	// Switchs
	void getStates(bool *states);
	byte getByte(void);

	// Neopixels
	void loading(int value);
	void chenillard(Colorled color = NONE);
	void flagChenillard();
	void stateOnLED(Colorled color = NONE);
	void setBrightness(uint8_t b);
	void setPixels(uint32_t *pixels);
	void setPixels(uint16_t pixels, Colorled color = NONE);
	void setSinglePixel(uint8_t pixelPos,  Colorled color);
	void setSinglePixel(uint8_t pixelPos,  uint8_t r = 0, uint8_t g = 0, uint8_t b = 0);
	void clear();
	void show();

#ifdef _DEBUG_
	void debug(void) const;
#endif

	int getNbSwitch(void) const;

private:
	uint8_t m_data;
	uint8_t m_clk;
	uint8_t m_latch;
	uint8_t m_led;

	int m_nbSwitch;
	int m_nbShift;
	int m_nbPixels;

	byte *m_states;

	Adafruit_NeoPixel m_pixels;

	void loadStates(void);
	uint32_t sinColor(unsigned long i);
};

#endif
