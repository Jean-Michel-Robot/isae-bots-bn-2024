#include <Motors.hpp>

#include <defines.hpp>


// #include "main_loop.hpp"

SoftwareSerial odrive_serial(ODRIVE_RX_PIN, ODRIVE_TX_PIN);

ODriveArduino odrive(odrive_serial);


void motors_init() {

    odrive_serial.begin(115200);

    // int axis = 0;

    // odrive_serial << "w axis" << axis << ".controller.config.vel_limit " << 10.0f << '\n';
}

void sendMotorCommand(int motor_number, float velCmd) {

    //TODO : transform velCmd into odrive command (nb_turn/s)
    // knowing the wheel diameter and the transmission ratio
    float odrv_cmd = velCmd*TRANSMISSION_RATIO/(PI*WHEEL_DIAMETER);

    // constrain the motor command for safety
    // if (abs(odrv_cmd) > 10) {
    //     p_ros->logPrint(ERROR, "Valeur de commande Odrive supérieure au seuil");
    // }
    odrv_cmd = constrain(odrv_cmd, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);


    // send value or its opposite because motors are symmetrical
    if (motor_number == BR_LEFT) {odrive.SetVelocity(motor_number, -odrv_cmd);}
    else {odrive.SetVelocity(motor_number, odrv_cmd);}

    //TOTEST with current feedforward
    // float current_feedforward = 0.0;
    // odrive.SetVelocity(motor_number, odrv_cmd, current_feedforward);

}

