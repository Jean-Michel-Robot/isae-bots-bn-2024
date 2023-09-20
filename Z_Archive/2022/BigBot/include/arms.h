#ifndef ARM_H
#define ARM_H

#include <Servo.h>
#include "bumpers.h"
#include "pumps.h"
#include <stdlib.h>

class Arm {
    private:

    public:
        void setup(int arm_angle_up, int arm_angle_down, int arm_angle_down_gallery);
        void loop();

        Servo* m_p_servo;
        Bumper* m_p_bumper;
        Pump* m_p_pump;

        int arm_angle_up;
        int arm_angle_down;
        int arm_angle_down_gallery;

        int triggered_take = 0;
        int movedown_take = 0;
        int goingdown_take = 0;
        int moveup_take = 0;


        int triggered_rack = 0;
        int movedown_rack = 0;
        int goingdown_rack = 0;
        int moveup_rack = 0;

        int triggered_gallery = 0;
        int movedown_gallery = 0;
        int goingdown_gallery = 0;
        int moveup_gallery = 0;

        int triggered_camp = 0;
        int movedown_camp = 0;
        int goingdown_camp = 0;
        int moveup_camp = 0;

    // Declares the arm
    Arm(int pin, Servo* m_p_servo_arg, Bumper* m_p_bumper_arg, Pump* m_p_pump);
 
    int current_angle;
    void move_down_take();
    void move_down_rack();
    void move_down_gallery();
    void move_down_camp();
};



#endif