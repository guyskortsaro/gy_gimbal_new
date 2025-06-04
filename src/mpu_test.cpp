#include <Arduino.h>
#include <MPU6050.h>
#include <SimpleFOC.h>
#include <Wire.h>
#include <math.h>

////////////////////////////////////////////
// ---------- IMU Configuration ---------- //
////////////////////////////////////////////
MPU6050 mpu;

float       gyroBiasX = 0.0, gyroBiasY = 0.0;
float       roll         = 0.0;
float       pitch        = 0.0;
float       gyroRollRate = 0.0, gyroPitchRate = 0.0;
float       prevPitchRate  = 0.0;
const float alpha          = 0.98;
const float GYRO_SCALE     = 131.0;
const float GYRO_LPF_ALPHA = 0.95;

////////////////////////////////////////////
// --------- Timing Variables ------------ //
////////////////////////////////////////////
unsigned long lastTime = 0, currentTime = 0;
float         dt = 0.0;

////////////////////////////////////////////
// ----------- Debugging Vars ----------- //
////////////////////////////////////////////
int debugger_index = 0;

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

void Gyro_Bias_Calibration(int samples = 360)
{
    long gxSum = 0, gySum = 0;

    Serial.println("Calibrating gyroscope... Keep the sensor still.");

    for (int i = 0; i < samples; i++)
    {
        gxSum += mpu.getRotationX();
        gySum += mpu.getRotationY();
        delay(2);
    }

    gyroBiasX = gxSum / (float)samples;
    gyroBiasY = gySum / (float)samples;

    Serial.print("Gyro bias X: ");
    Serial.println(gyroBiasX);
    Serial.print("Gyro bias Y: ");
    Serial.println(gyroBiasY);

    lastTime = millis(); // Initialize timer
}

////////////////////////////////////////////
// ---------- Sensor Processing ---------- //
////////////////////////////////////////////
void Pitch_And_Roll_Calculation()
{
    // Read raw data
    int16_t ax = mpu.getAccelerationX();
    int16_t ay = mpu.getAccelerationY();
    int16_t az = mpu.getAccelerationZ();
    int16_t gx = mpu.getRotationX();
    int16_t gy = mpu.getRotationY();

    // Time delta in seconds
    currentTime = millis();
    dt          = (float)(currentTime - lastTime) / 1000.0f;
    if (dt <= 0.0)
        dt = 0.001;
    lastTime = currentTime;

    // Convert accelerometer to float
    float axf = (float)ax;
    float ayf = (float)ay;
    float azf = (float)az;

    // Accel-derived angles
    float accelRoll  = atan2(ayf, azf);                          // around X
    float accelPitch = atan2(-axf, sqrt(ayf * ayf + azf * azf)); // around Y

    // --- Filter time check ---
    if (dt < 0.001f)
        dt = 0.001f;
    if (dt > 0.05f)
        dt = 0.05f;

    // --- Raw gyro conversion ---
    float rawGyroRoll  = (gx - gyroBiasX) / GYRO_SCALE * DEG_TO_RAD;
    float rawGyroPitch = (gy - gyroBiasY) / GYRO_SCALE * DEG_TO_RAD;

    // --- LPF smoothing ---
    gyroRollRate  = GYRO_LPF_ALPHA * gyroRollRate + (1 - GYRO_LPF_ALPHA) * rawGyroRoll;
    gyroPitchRate = GYRO_LPF_ALPHA * gyroPitchRate + (1 - GYRO_LPF_ALPHA) * rawGyroPitch;

    // --- Complementary filter ---
    roll  = alpha * (roll + gyroRollRate * dt) + (1 - alpha) * accelRoll;
    pitch = alpha * (pitch + gyroPitchRate * dt) + (1 - alpha) * accelPitch;
}

////////////////////////////////////////////
// ------------- Debug Output ------------ //
////////////////////////////////////////////
void Debbuging_Print()
{
    Serial.print(millis());
    Serial.print(",");
    Serial.print(roll, 4);
    Serial.print(",");
    Serial.print(pitch, 4);
    Serial.print(",");
    Serial.print(gyroRollRate, 4);
    Serial.print(",");
    Serial.println(gyroPitchRate, 4);
}

void setup()
{
    Serial.begin(115200);
    delay(1000);
    Initialize_IMU();
    Gyro_Bias_Calibration();
}

void loop()
{
    Pitch_And_Roll_Calculation();
    Debbuging_Print();
}