#include "Roll_init.h"

// Define the actual objects
BLDCDriver3PWM ROLL_driver(ROLL_PWM_A, ROLL_PWM_B, ROLL_PWM_C, ROLL_ENABLE_PIN);
BLDCMotor ROLL_motor(11);
MagneticSensorSPI ROLL_encoder = MagneticSensorSPI(11, 14, 0x3FFF); // SCN pin is 11
