#include <Arduino.h>
#include "robot.h"
#include "isb.h"
#include "UltrasonicRos.h"

#include "src/ISB/IsbBasicRos.h"
#include "src/ISB/IsbMovingRos.h"

#include "parameters.h"

/*
Régler les angles max
Gestion d'erreur en cas de perte en remontant + analyse situation
*/


int pin_pump0 = 2; // 20
int pin_pump1 = 21; // 21
int pin_pump2 = 3;
int pin_pump3 = 2; // 2

int pin_bumper0 = 22;
int pin_bumper1 = 23;
int pin_bumper2 = 1;
int pin_bumper3 = 0; // pour l'isb

int pin_servo0 = 18;
int pin_servo1 = 19;
int pin_servo2 = 5;
int pin_servo3 = 4;

int initial_angle0 = 90;
int initial_angle1 = 90;
int initial_angle2 = 70;
int initial_angle3 = 130;

Pump m_pump0(pin_pump0);
Pump m_pump1(pin_pump1);
Pump m_pump2(pin_pump2);
Pump m_pump3(pin_pump3);

Bumper m_bumper0(pin_bumper0);
Bumper m_bumper1(pin_bumper1);
Bumper m_bumper2(pin_bumper2);
Bumper m_bumper3(pin_bumper3);

Servo m_p_servo0 = Servo();
Servo m_p_servo1 = Servo();
Servo m_p_servo2 = Servo();
Servo m_p_servo3 = Servo();

Arm m_arm0(pin_servo0, &m_p_servo0, &m_bumper0, &m_pump0);
Arm m_arm1(pin_servo1, &m_p_servo1, &m_bumper1, &m_pump1);
Arm m_arm2(pin_servo2, &m_p_servo2, &m_bumper2, &m_pump2);
Arm m_arm3(pin_servo3, &m_p_servo3, &m_bumper3, &m_pump3);

ros::NodeHandle ros_node;

Arm* arm_array[4] = {&m_arm0, &m_arm1, &m_arm2, &m_arm3};

// 2 : Grand en haut et faible en bas
// 1 : Faibe en haut et grand en bas
// 3 : Faible en haut et grand en bas
// 4 : Grand en haut et faible en bas

int arm_angle_up[4] = {55, 105, 65, 180}; // 55, 120, 40, 180
int arm_angle_down[4] = {145, 23, 151, 100};
int arm_angle_down_gallery[4] = {80, 90, 80, 150};

Robot m_robot(&ros_node, arm_array);

Isb m_isb(&ros_node, pin_bumper3, &m_bumper3);

// timers :

int movedown_take_timer_ref = millis() + 1000*3600;
int movedown_gallery_timer_ref = millis() + 1000*3600;
int moveup_gallery_timer_ref = millis() + 1000*3600;
int movedown_camp_timer_ref = millis() + 1000*3600;

int timer_take = millis() + 1000*3600;
int timer_camp = millis() + 1000*3600;

int timer_camp_define = 0;
int timer_take_define = 0;

/* Sonars */
UltrasonicSensor *ultrasonicSensorR = new UltrasonicSensor("R", SONAR_PIN_1, SONAR_BASE_1, SONAR_COEFF_1, SONAR_TIME_BEFORE_REFRESH);
UltrasonicSensor *ultrasonicSensorL = new UltrasonicSensor("L", SONAR_PIN_2, SONAR_BASE_2, SONAR_COEFF_2, SONAR_TIME_BEFORE_REFRESH);

UltrasonicSensor *ultrasonicSensorArray[] = {ultrasonicSensorR, ultrasonicSensorL};

UltrasonicRos ultrasonicRos = UltrasonicRos(&ros_node, SONAR_ROS_NB_ULTRASONIC_SENSORS, ultrasonicSensorArray, SONAR_ROS_TIME_PUB_DISTANCES);

void setup()
{
  ros_node.initNode();


  /* Sonars */
  for (int i = 0; i < SONAR_ROS_NB_ULTRASONIC_SENSORS; i++)
    {
      ultrasonicSensorArray[i]->setup();
    }

    ultrasonicRos.setup();

  m_robot.setup(arm_angle_up, arm_angle_down, arm_angle_down_gallery);

  m_isb.setup();

  m_isb.isb_state = m_isb.tirette->state;

}

void loop()
{
  ros_node.spinOnce();

  m_isb.update_state_tirette();
  m_isb.new_isb_state = m_isb.tirette->state;

  if (m_isb.isb_state == 1 and m_isb.new_isb_state == 0) {
    m_isb.msg_tirette.data = 1;
    m_isb.m_publisher_tirette.publish(&(m_isb.msg_tirette));
  }

  m_isb.isb_state = m_isb.new_isb_state;

  /* Ultrasonic sonars */
  for (int i = 0; i < SONAR_ROS_NB_ULTRASONIC_SENSORS; i++)
  {
    ultrasonicSensorArray[i]->loop();
  }

  ultrasonicRos.loop();
  
  for(int i = 0; i<4; i++) {
  
    // Move down :
    if (arm_array[i]->movedown_take == 1 && arm_array[i]->triggered_take == 1) {

      arm_array[i]->current_angle = arm_array[i]->arm_angle_down;
      arm_array[i]->m_p_servo->write(arm_array[i]->current_angle);

      arm_array[i]->movedown_take = 0;
      arm_array[i]->goingdown_take = 1;

      movedown_take_timer_ref = millis();
    }

    int movedown_take_timer = millis() - movedown_take_timer_ref;
    arm_array[i]->m_p_bumper->update_state();
    if (arm_array[i]->m_p_bumper->state == 1 && arm_array[i]->triggered_take == 1 && arm_array[i]->goingdown_take == 1) {
      
      arm_array[i]->m_p_pump->switch_pump(1);
      
      if (timer_take_define == 0) {
        timer_take = millis();
        timer_take_define = 1;
      }

      if (millis() - timer_take > 1000) {
        arm_array[i]->goingdown_take = 0;
        timer_take_define = 0;
        arm_array[i]->moveup_take = 1;
      }     
    }

    if (arm_array[i]->moveup_take == 1 && arm_array[i]->triggered_take == 1) {

      arm_array[i]->current_angle = arm_array[i]->arm_angle_up;
      arm_array[i]->m_p_servo->write(arm_array[i]->current_angle); 
      
      arm_array[i]->moveup_take = 0;
      arm_array[i]->triggered_take = 0;

      m_robot.msg_to_pub.data = 0;
      m_robot.m_publisher_send.publish(&(m_robot.msg_to_pub));
    }

    if (arm_array[i]->m_p_bumper->state == 0 && arm_array[i]->triggered_take == 1 && arm_array[i]->goingdown_take == 1 && movedown_take_timer > 1000) {
      
      arm_array[i]->current_angle = arm_array[i]->arm_angle_up;
      arm_array[i]->m_p_servo->write(arm_array[i]->current_angle);

      arm_array[i]->goingdown_take = 0;
      arm_array[i]->moveup_take = 0;
      arm_array[i]->triggered_take = 0;

      m_robot.msg_to_pub.data = 1;
      m_robot.m_publisher_send.publish(&(m_robot.msg_to_pub));
    }

    // Move camp
    if (arm_array[i]->movedown_camp == 1 && arm_array[i]->triggered_camp == 1) {

      arm_array[i]->current_angle = arm_array[i]->arm_angle_down;
      arm_array[i]->m_p_servo->write(arm_array[i]->current_angle);

      arm_array[i]->movedown_camp = 0;
      arm_array[i]->goingdown_camp = 1;

      movedown_camp_timer_ref = millis();
    }

    int movedown_camp_timer = millis() - movedown_camp_timer_ref;
    if (arm_array[i]->triggered_camp == 1 && arm_array[i]->goingdown_camp == 1 && movedown_camp_timer > 500) {
            
      arm_array[i]->m_p_pump->switch_pump(0);
      
      if (timer_camp_define == 0) {
        timer_camp = millis();
        timer_camp_define = 1;
      }

      if (millis() - timer_camp > 500) {
        arm_array[i]->moveup_camp = 1;
        arm_array[i]->goingdown_camp = 0;
        timer_camp_define = 0;
      }
    }

    if (arm_array[i]->moveup_camp == 1 && arm_array[i]->triggered_camp == 1) {

      arm_array[i]->current_angle = arm_array[i]->arm_angle_up;
      arm_array[i]->m_p_servo->write(arm_array[i]->current_angle);
      
      arm_array[i]->moveup_camp = 0;
      arm_array[i]->triggered_camp = 0;

      m_robot.msg_to_pub.data = 0;
      m_robot.m_publisher_send.publish(&(m_robot.msg_to_pub));
    }

    // Move gallery

    if (arm_array[i]->movedown_gallery == 1 && arm_array[i]->triggered_gallery == 1) {

      arm_array[i]->current_angle = arm_array[i]->arm_angle_down_gallery;
      arm_array[i]->m_p_servo->write(arm_array[i]->current_angle);

      arm_array[i]->movedown_gallery = 0;
      arm_array[i]->goingdown_gallery = 1;

      movedown_gallery_timer_ref = millis();
      moveup_gallery_timer_ref = millis();
    }

    int moveup_gallery_timer = millis() - moveup_gallery_timer_ref;
    if (arm_array[i]->moveup_gallery == 1 && arm_array[i]->triggered_gallery == 1 && moveup_gallery_timer > 1000) {
      
      arm_array[i]->current_angle = arm_array[i]->arm_angle_up;
      arm_array[i]->m_p_servo->write(arm_array[i]->arm_angle_up);

      arm_array[i]->moveup_gallery = 0;
      arm_array[i]->triggered_gallery = 0;

      m_robot.msg_to_pub.data = 0;
      m_robot.m_publisher_send.publish(&(m_robot.msg_to_pub));
    }

    int movedown_gallery_timer = millis() - movedown_gallery_timer_ref;
    if (arm_array[i]->triggered_gallery == 1 && arm_array[i]->goingdown_gallery == 1 && movedown_gallery_timer > 500) {
      
      arm_array[i]->goingdown_gallery = 0;
      arm_array[i]->m_p_pump->switch_pump(0);
      arm_array[i]->moveup_gallery = 1;
    }


  
    
  //delay(100);


  }
}