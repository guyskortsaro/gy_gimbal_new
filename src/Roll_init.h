#ifndef ROLL_INIT_H
#define ROLL_INIT_H

#include <Arduino.h>
#include <SimpleFOC.h>

// Define ROLL Motor Driver PINS
#define ROLL_PWM_A 9
#define ROLL_PWM_B 10
#define ROLL_PWM_C 11
#define ROLL_ENABLE_PIN 3

// Declare the objects (extern keyword allows global access across files)
extern BLDCDriver3PWM ROLL_driver;
extern BLDCMotor ROLL_motor;
//extern MagneticSensorSPI ROLL_encoder;

#endif
