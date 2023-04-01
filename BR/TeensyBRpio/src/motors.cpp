#include "motors.hpp"

SoftwareSerial odrive_serial(ODRIVE_RX_PIN, ODRIVE_TX_PIN);

ODriveArduino odrive(odrive_serial);


void motors_init() {

    odrive_serial.begin(115200);

    // int axis = 0;

    // odrive_serial << "w axis" << axis << ".controller.config.vel_limit " << 10.0f << '\n';
}

void sendMotorCommand(int motor_number, float velCmd) {

    //TODO : transform velCMd into odrive command
    float odrv_cmd = 2 * velCmd / 60.0;  // command between -255 and 255 (extreme values)

    if (motor_number == 1) {odrive.SetVelocity(motor_number, -odrv_cmd);}
    else {odrive.SetVelocity(motor_number, odrv_cmd);}

    // OR
    // float current_feedforward = 0.0;
    // odrive.SetVelocity(motor_number, odrv_cmd, current_feedforward);

}

