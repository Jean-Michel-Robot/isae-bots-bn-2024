/* ODOS*/

#define ODOS_METHOD 1
#define ODO_HARD


/* ASSERV */
#define WHEEL_DISTANCE 0.25 //m


// Macros
#define sign(value) (value > 0 ? 1 : -1)

// Parameters
#define ODRIVE_RX_PIN 0
#define ODRIVE_TX_PIN 1

#define ODO_SEND_POSITION_TIMER 100 //ms

// Ramps
#define ACCEL_BRAKE 0.5  // m/s^-2
#define RAMP_EPSILON 0.001  // epsilon for values to be considered the same