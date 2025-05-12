#ifndef PITCH_INIT_H
#define PITCH_INIT_H

#include <Arduino.h>
#include <SimpleFOC.h>

// Define PITCH Motor Driver PINS
#define PITCH_PWM_A 2
#define PITCH_PWM_B 3
#define PITCH_PWM_C 4
#define PITCH_ENABLE_PIN 8

// Declare PITCH driver, motor, and encoder as extern so they're usable elsewhere
extern BLDCDriver3PWM PITCH_driver;
extern BLDCMotor PITCH_motor;
extern MagneticSensorSPI PITCH_encoder;

#endif
