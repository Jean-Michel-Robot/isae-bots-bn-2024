
#include "a_define.hpp"
#include "Deguisement.hpp"

Adafruit_NeoPixel* DeguisementROS::m_p_neopixel = NULL;

DeguisementROS::DeguisementROS(ros::NodeHandle* p_nh) :
    m_sub("/strat/deguisement", DeguisementROS::subCallback){
        m_p_neopixel = new Adafruit_NeoPixel(DEGUISEMENT_NEOPX_NUMBER, DEGUISEMENT_NEOPX_PIN, NEO_RGB + NEO_KHZ800);
        m_p_nh = p_nh;
}

void DeguisementROS::setup(){
    m_p_neopixel->begin();
    m_p_neopixel->clear();

    m_p_nh->subscribe(m_sub);
}

void DeguisementROS::loop(){
    ;
}

void DeguisementROS::subCallback(const std_msgs::Int16& stateVal){
 for (int pixel = 0; pixel < DEGUISEMENT_NEOPX_NUMBER; pixel++) { // for each pixel
    m_p_neopixel->setPixelColor(pixel, m_p_neopixel->Color(0, 255, 0)); // it only takes effect if pixels.show() is called
    m_p_neopixel->show();  
 }
}