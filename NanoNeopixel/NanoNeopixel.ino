#include <Adafruit_NeoPixel.h>
#include <ros.h>
#include <std_msgs/Int16.h>

#define PIXEL_PIN 3
#define NUM_PIXELS 29

Adafruit_NeoPixel pixels(NUM_PIXELS, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

void lightCallback(const std_msgs::Int16& msg);

ros::NodeHandle nh;
std_msgs::Int16 light_msg;
ros::Subscriber<std_msgs::Int16> light_sub("/strat/deguisement", lightCallback);

bool up_state = false;
bool blink_state = false;



void setup() {
  pixels.begin();
  nh.initNode();
  nh.subscribe(light_sub);
}

void loop() {
  nh.spinOnce();
  if(up_state){
    if(blink_state){
       for (int i = 0; i < NUM_PIXELS; i++) {
        pixels.setPixelColor(i, pixels.Color(255, 0, 127)); // PInk color
        blink_state = false;
      }
    }
    else{
       for (int i = 0; i < NUM_PIXELS; i++) {
        pixels.setPixelColor(i, pixels.Color(0, 255, 255)); // Cyan color
        blink_state = true;
      }
    }
  }
  else{
     for (int i = 0; i < NUM_PIXELS; i++) {
        pixels.setPixelColor(i, pixels.Color(255, 255, 255)); // PInk color
      }
  }
  pixels.show();
  delay(333); 
}

void lightCallback(const std_msgs::Int16& msg) {
  if (msg.data == 1) {
    // Turn on the NeoPixels
    up_state = true;;
  } else {
    // Turn off the NeoPixels
    for (int i = 0; i < NUM_PIXELS; i++) {
      pixels.setPixelColor(i, pixels.Color(0, 0, 0)); // Black color
      up_state = false;
      
    }
    pixels.show();
  }
}
