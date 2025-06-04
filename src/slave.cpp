#include <Arduino.h>
#include <MPU6050.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <math.h>
#include "Roll_init.h"
#include "myPID.h"
#include <SimpleFOC.h>

// --------- UART Configuration ---------- //
SoftwareSerial fromA(5, 6); // Rx,Tx pins

// --- Velocity Variable Configuration --- //
float target_roll_velocity = 0;

void Initialize_Drivers()
{
    ROLL_driver.voltage_power_supply = 12;
    ROLL_driver.init();
}
void Linking_Drivers_And_Encoders_To_Motors()
{
    ROLL_motor.linkDriver(&ROLL_driver);
}
void Setting_Motors_Modes()
{
    ROLL_motor.controller  = MotionControlType::velocity_openloop;
}
void Initialize_Motors()
{
    ROLL_motor.init();
    ROLL_motor.initFOC();
}
void Communication(){
        if (fromA.available())
    {
        String data = fromA.readStringUntil('\n');
        if (data.startsWith("R:"))
        {
            target_roll_velocity = data.substring(2).toFloat();
            Serial.print(millis());
            Serial.print(",");
            Serial.println(target_roll_velocity, 4);
        }
    }
}
void setup()
{
    Serial.begin(115200);
    fromA.begin(115200);
    Initialize_Drivers();
    Linking_Drivers_And_Encoders_To_Motors();
    Setting_Motors_Modes();
    Initialize_Motors();
}
void loop()
{
    Communication();
    ROLL_motor.move(target_roll_velocity);
}