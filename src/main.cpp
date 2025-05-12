#include "Pitch_init.h"
#include "Roll_init.h"
#include "myPID.h"
#include <Arduino.h>
#include <MPU6050.h>
#include <SimpleFOC.h>
#include <Wire.h>
#include <math.h>

// Define PID  variables
myPID pitchPID(0.1, 0.0, 0.0);
myPID rollPID(0.1, 0.0, 0.0);
float PitchPID_output       = 0.0;
float RollPID_output        = 0.0;
float Pitch_target_velocity = 0.0;
float Roll_target_velocity  = 0.0;

// Define IMU
MPU6050 mpu;

// Define Time Variables
unsigned long lastTime = 0, currentTime = 0;
float         dt = 0.0;

// Define IMU Variables
float gyroBiasX = 0.0, gyroBiasY = 0.0, pitch = 0.0, roll = 0.0;

// Define IMU Complimentary Filter Variables
const float pitch_alpha   = 0.96;
const float roll_alpha    = 0.96;
float       gyroPitchRate = 0.0, gyroRollRate = 0.0;
float       prevPitchRate = 0.0;

// Define debugger index counter
int debugger_index = 0;

// Steady state variables
bool        pitch_in_steady_state              = false;
const float pitch_steady_state_enter_threshold = 0.01;
const float pitch_steady_state_exit_threshold  = 0.03;
bool        roll_in_steady_state               = false;
const float roll_steady_state_enter_threshold  = 0.01;
const float roll_steady_state_exit_threshold   = 0.03;

// Setup Functions
void Initialize_Encoders()
{
    PITCH_encoder.init();
    Serial.println("Pitch Encoder is initialized");
    ROLL_encoder.init();
    Serial.println("Roll Encoder is initialized");
}
void Initialize_Drivers()
{
    PITCH_driver.voltage_power_supply = 18;
    PITCH_driver.init();
    ROLL_driver.voltage_power_supply = 18;
    ROLL_driver.init();
}
void Linking_Drivers_And_Encoders_To_Motors()
{
    PITCH_motor.linkDriver(&PITCH_driver);
    PITCH_motor.linkSensor(&PITCH_encoder);
    ROLL_motor.linkDriver(&ROLL_driver);
    ROLL_motor.linkSensor(&ROLL_encoder);
}
void Setting_Motors_Modes()
{
    PITCH_motor.controller = MotionControlType::velocity_openloop;
    ROLL_motor.controller  = MotionControlType::velocity_openloop;
}
void Initialize_Motors()
{
    PITCH_motor.init();
    PITCH_motor.initFOC();
    ROLL_motor.init();
    ROLL_motor.initFOC();
}
void Initialize_IMU()
{
    Wire.begin();
    mpu.initialize();
    if (!mpu.testConnection())
    {
        Serial.println("MPU6050 connection failed!");
        while (1)
            ;
    }
    Serial.println("MPU6050 initialized.");
    delay(1000);
}
void Gyro_Bias_Calibration(int samples = 500)
{
    long gxSum = 0, gySum = 0;
    Serial.println("Calibrating gyroscope... Keep the sensor still.");
    for (int i = 0; i < samples; i++)
    {
        gxSum += mpu.getRotationX();
        gySum += mpu.getRotationY();
        delay(2);
    }
    gyroBiasX = -gxSum / (float)samples;
    gyroBiasY = -gySum / (float)samples;
    Serial.print("Gyro bias X: ");
    Serial.println(gyroBiasX);
    Serial.print("Gyro bias Y: ");
    Serial.println(gyroBiasY);
    lastTime = millis(); // Initialize timer
}

void Pitch_And_Roll_Calculation()
{
    // Read raw data
    int16_t ax = mpu.getAccelerationX();
    int16_t ay = mpu.getAccelerationY();
    int16_t az = mpu.getAccelerationZ();
    int16_t gx = mpu.getRotationX();
    int16_t gy = mpu.getRotationY();

    // Time difference in seconds
    currentTime = millis();
    dt          = (float)(currentTime - lastTime) / 1000.0f;
    lastTime    = currentTime;

    // Convert accel to float
    float axf = (float)ax;
    float ayf = (float)ay;
    float azf = (float)az;

    // Accel-derived angles (in radians)
    float accelRoll  = atan2(ayf, azf);                              // around X
    float accelPitch = atan2(-axf, sqrt(ayf * ayf + azf * azf));    // around Y

    // Gyro rates (rad/s) after bias correction
    gyroRollRate  = (gx - gyroBiasX) / 131.0 * DEG_TO_RAD;
    gyroPitchRate = (gy - gyroBiasY) / 131.0 * DEG_TO_RAD;

    // Complementary filter
    roll  = roll_alpha * (roll + gyroRollRate * dt) + (1 - roll_alpha) * accelRoll;
    pitch = pitch_alpha * (pitch + gyroPitchRate * dt) + (1 - pitch_alpha) * accelPitch;
}

void Pitch_steady_state()
{
    if (pitch_in_steady_state)
    {
        if (abs(pitchPID.getError()) > pitch_steady_state_exit_threshold)
        {
            pitch_in_steady_state = false;
        }
    }
    else
    {
        if (abs(pitchPID.getError()) < pitch_steady_state_enter_threshold)
        {
            pitch_in_steady_state = true;
        }
    }
    if (pitch_in_steady_state)
    {
        Pitch_target_velocity = 0.0;
    }
    else
    {
        Pitch_target_velocity = PitchPID_output / dt;
    }
}
void Roll_steady_state()
{
    if (roll_in_steady_state)
    {
        if (abs(rollPID.getError()) > roll_steady_state_exit_threshold)
        {
            roll_in_steady_state = false;
        }
    }
    else
    {
        if (abs(rollPID.getError()) < roll_steady_state_enter_threshold)
        {
            roll_in_steady_state = true;
        }
    }

    if (roll_in_steady_state)
    {
        Roll_target_velocity = 0.0;
    }
    else
    {
        Roll_target_velocity = RollPID_output / dt;
    }
}
void PID_calculation()
{
    PitchPID_output = pitchPID.compute(0.0, pitch, dt);
    RollPID_output  = rollPID.compute(0.0, roll, dt);
}
void Debbuging_Print()
{
    debugger_index++;
    if (debugger_index == 100)
    {
        Serial.print("Pitch: ");
        Serial.print(pitch, 2);
        Serial.print(" Rad      Pitch Rate: ");
        Serial.print(gyroPitchRate, 2);
        Serial.print(" Rad/s      Pitch Error: ");
        Serial.print(pitchPID.getError(), 2);
        Serial.print(" Rad      Pitch Target Velocity: ");
        Serial.print(Pitch_target_velocity, 2);
        Serial.print(" Rad/s      roll: ");
        Serial.print(roll, 2);
        Serial.print(" Rad      Roll Rate: ");
        Serial.print(gyroRollRate, 2);
        Serial.print(" Rad/s      Roll Error");
        Serial.print(rollPID.getError(), 2);
        Serial.print(" Rad/s      Roll Target Velocity: ");
        Serial.print(Roll_target_velocity, 2);
        Serial.println(" Rad/s");
        debugger_index = 0;
    }
}

void setup()
{
    Serial.begin(115200);
    delay(1000);
    Initialize_Encoders();
    Initialize_Drivers();
    Linking_Drivers_And_Encoders_To_Motors();
    Setting_Motors_Modes();
    Initialize_Motors();
    Initialize_IMU();
    Gyro_Bias_Calibration();
}

void loop()
{
    Pitch_And_Roll_Calculation();
    PID_calculation();
    Pitch_steady_state();
    Roll_steady_state();
    Debbuging_Print();

    // Moving motor
    PITCH_motor.move(Pitch_target_velocity);
    ROLL_motor.move(Roll_target_velocity);
}
