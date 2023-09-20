#include "ISB.h"

ISB::ISB(uint8_t data, uint8_t clk, uint8_t latch, uint8_t led)
	: m_data(data), m_clk(clk), m_latch(latch), m_led(led), m_nbSwitch(NB_SWITCH), m_nbPixels(NB_PIXELS),
	m_pixels(m_nbPixels, m_led, NEO_GRB + NEO_KHZ800)
{
	m_nbShift = (int)(m_nbSwitch / 8. + .5);
}

ISB::ISB(uint8_t data, uint8_t clk, uint8_t latch, uint8_t led, int nbSwitch, int nbPixels)
	: m_data(data), m_clk(clk), m_latch(latch), m_led(led), m_nbSwitch(nbSwitch), m_nbPixels(nbPixels),
	m_pixels(m_nbPixels, m_led, NEO_GRB + NEO_KHZ800)

{
	m_nbShift = (int)(m_nbSwitch / 8. + .5);
}

ISB::~ISB(void)
{
	free(m_states);
}

// ################### PUBLIC FUNCTIONS ###################
int ISB::begin(void)
{
	pinMode(m_data, INPUT);
	pinMode(m_clk, OUTPUT);
	pinMode(m_latch, OUTPUT);

	m_pixels.begin();
	m_pixels.show();

	m_states = (byte *)calloc(m_nbShift, sizeof(byte));
	if (m_states == NULL)
		return -1;

	return 0;
}

void ISB::getStates(bool *states)
{
	loadStates();

	for (int i = 0; i < m_nbSwitch; i++)
		states[i] = (m_states[i / 8] & (1 << (i % 8)) ? true : false);
}

byte ISB::getByte(void)
{
	loadStates();
	return m_states[0];
}

void ISB::loading(int value)
{
	int last = (int)(value * m_nbPixels / 100);

	uint16_t pixels = 0;
	for (int i = 0; i < last; i++) {
		pixels |= 1 << i;
	}
	setPixels(pixels, RED);

	if (last < m_nbPixels)
	{
		int intensity = (value * m_nbPixels - last * 100) * 255 / 100;
		setSinglePixel(last, intensity, 0, 0);
	}

	m_pixels.show();
}

void ISB::stateOnLED(Colorled color)
{
	byte state = getByte();

	uint16_t pixels = 0;

	for (int i = 0; i < m_nbSwitch && i < m_nbPixels; i++)
	{
		if (state & (1 << (i % 8)))
		{
			pixels |= 1 << i;		
		}
	}

	setPixels(pixels, color);
	m_pixels.show();
}

void ISB::clear()
{
	m_pixels.clear();
	m_pixels.show();
}

void ISB::flagChenillard()
{
	static int phase = 0;
	static unsigned long lastT = millis();
	static int seg1 = (int)(m_nbPixels / 3);
	static int seg2 = m_nbPixels - 1 - seg1;

	if (millis() - lastT > 150)
	{
		lastT = millis();
		phase++;
	}

	if (phase >= m_nbPixels * 2)
		phase = 0;

	for (int i = 0; i < m_nbPixels; i++)
	{
		if ((phase < m_nbPixels && i > phase) || (phase >= m_nbPixels && i <= phase - m_nbPixels))
		{
			setSinglePixel(i, 0, 0, 0);
		}
		else
		{

			Colorled color = NONE;
			if (i <= seg1)
				color = BLUE;
			else if (i >= seg2)
				color = RED;
			else if (i > seg1 && i < seg2)
			{
				color = WHITE;
			}

			setSinglePixel(i, color);
		}
	}

	m_pixels.show();
}

void ISB::chenillard(Colorled color)
{
	static int i = 0;
	static unsigned long lastT = millis();

	setPixels(1 << i, color);

	if (lastT + TEMPO_CHEN < millis())
	{
		i++;
		if (i >= m_nbPixels)
			i = 0;
		lastT = millis();
	}
	m_pixels.show();
}

void ISB::setBrightness(uint8_t b)
{
	m_pixels.setBrightness(b);
}

void ISB::setPixels(uint32_t *pixels)
{
	m_pixels.clear();
	for (int i = 0; i < m_nbPixels; i++)
	{
		m_pixels.setPixelColor(i, pixels[i]);
	}

}

void ISB::setPixels(uint16_t pixels, Colorled color)
{
	m_pixels.clear();
	for (int i = 0; i < m_nbPixels; i++)
	{
		if (pixels & (1 << (m_nbPixels - 1 - i)))
		{
			switch (color)
			{
			case RED:
				m_pixels.setPixelColor(i, 255, 0, 0);
				break;
			case GREEN:
				m_pixels.setPixelColor(i, 0, 255, 0);
				break;
			case BLUE:
				m_pixels.setPixelColor(i, 0, 0, 255);
				break;
			case ORANGE:
				m_pixels.setPixelColor(i, 255, 50, 0);
				break;
			case PURPLE:
				m_pixels.setPixelColor(i, 255, 0, 255);
				break;
			case YELLOW:
				m_pixels.setPixelColor(i, 255, 130, 0);
				break;
			case WHITE:
				m_pixels.setPixelColor(i, 255, 255, 255);
				break;
			case NONE:
				m_pixels.setPixelColor(i, 0, 0, 0);
				break;
			default:
				m_pixels.setPixelColor(i, sinColor(millis()));
				break;
			}
		}
	}

}

void ISB::setSinglePixel(uint8_t pixelPos,  Colorled color)
{
	int led = m_nbPixels - 1 - pixelPos;
	switch (color)
	{
	case RED:
		m_pixels.setPixelColor(led, 255, 0, 0);
		break;
	case GREEN:
		m_pixels.setPixelColor(led, 0, 255, 0);
		break;
	case BLUE:
		m_pixels.setPixelColor(led, 0, 0, 255);
		break;
	case ORANGE:
		m_pixels.setPixelColor(led, 255, 50, 0);
		break;
	case PURPLE:
		m_pixels.setPixelColor(led, 255, 0, 255);
		break;
	case YELLOW:
		m_pixels.setPixelColor(led, 255, 130, 0);
		break;
	case WHITE:
		m_pixels.setPixelColor(led, 255, 255, 255);
		break;
	case NONE:
		m_pixels.setPixelColor(led, 0, 0, 0);
		break;
	default:
		m_pixels.setPixelColor(led, sinColor(millis()));
		break;
	}


}

void ISB::setSinglePixel(uint8_t pixelPos,  uint8_t r, uint8_t g, uint8_t b)
{
	m_pixels.setPixelColor(m_nbPixels - 1 - pixelPos, r, g, b);

}

void ISB::show()
{
	m_pixels.show();
}

#ifdef _DEBUG_
void ISB::debug(void) const
{
	Serial.println(F("ISB: Debug()"));
	Serial.print(F("\tFile:\t"));
	Serial.println(__FILE__);
	Serial.print(F("\tCompiled on:\t"));
	Serial.print(__DATE__);
	Serial.print(F("  "));
	Serial.println(__TIME__);
	Serial.print(F("\tNb of switch:\t"));
	Serial.print(m_nbSwitch);
	Serial.print(F("\tNb of Registers:\t"));
	Serial.println(m_nbShift);
	Serial.print(F("\tNb of leds:\t"));
	Serial.println(m_nbPixels);

	Serial.print(F("\tLast states:\t"));
	for (int i = 0; i < m_nbShift; i++)
	{
		Serial.print(m_states[i], BIN);
		Serial.print(F(" "));
	}

	Serial.println("\n");
}
#endif

// ####################### ASSESSORS #######################
int ISB::getNbSwitch(void) const
{
	return m_nbSwitch;
}

// ################### PRIVATE FUNCTIONS ###################
void ISB::loadStates(void)
{
	// Load datas in register
	digitalWrite(m_latch, LOW);
	delayMicroseconds(LATCH_TEMPO);
	digitalWrite(m_latch, HIGH);

	for (int i = 0; i < m_nbShift; i++)
	{
		m_states[i] = 0b00000000;

		for (int j = 0; j < 8; j++)
		{
			digitalWrite(m_clk, LOW);
			delayMicroseconds(CLOCK_TEMPO);

			// The pin state is originaly set to 0
			if (digitalRead(m_data) == HIGH)
				m_states[i] = m_states[i] | (1 << j);

			digitalWrite(m_clk, HIGH);
			delayMicroseconds(CLOCK_TEMPO);
		}
	}
}

uint32_t ISB::sinColor(unsigned long i)
{
	return m_pixels.Color(abs(i % 510 - 255), abs((i + 170) % 510 - 255), abs((i + 340) % 510 - 255));
}
