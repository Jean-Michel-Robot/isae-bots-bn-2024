#ifndef _ARDUINO_LED_HPP_
#define _ARDUINO_LED_HPP_

class BlinkLED {
  public:
    BlinkLED();
    void loop();

  private:
    unsigned long m_timer;
    bool m_state;
};

#endif