#include "Pitch_init.h"

// Define global motor objects
BLDCDriver3PWM PITCH_driver(PITCH_PWM_A, PITCH_PWM_B, PITCH_PWM_C, PITCH_ENABLE_PIN);
BLDCMotor PITCH_motor(11);
MagneticSensorSPI PITCH_encoder = MagneticSensorSPI(12, 14, 0x3FFF); // SCN pin is 12
