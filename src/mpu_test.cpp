#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// Complementary filter constant
const float alpha = 0.96;

// Filtered angles
float pitch = 0.0;
float roll  = 0.0;

// Gyroscope bias
float gyroBiasX = 0.0;
float gyroBiasY = 0.0;

// Timing
unsigned long lastTime = 0;

// Function declarations
void calibrateGyroBias(int samples = 500);
void readRawData(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz);
void computeAngles(float dt, int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy);
void printDebug(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz, float dt);

void setup() {
    Serial.begin(115200);
    Wire.begin();
    mpu.initialize();

    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed!");
        while (1);
    }

    Serial.println("MPU6050 initialized. Stabilizing...");
    delay(1000);

    calibrateGyroBias();
    lastTime = micros(); // Use micros() for higher resolution
}

void loop() {
    int16_t ax, ay, az, gx, gy, gz;

    readRawData(ax, ay, az, gx, gy, gz);

    unsigned long currentTime = micros();
    float dt = (currentTime - lastTime) / 1e6;
    lastTime = currentTime;

    computeAngles(dt, ax, ay, az, gx, gy);
    printDebug(ax, ay, az, gx, gy, gz, dt);

    delay(100); // Slow down serial output for readability
}

void calibrateGyroBias(int samples) {
    long sumX = 0, sumY = 0;
    Serial.println("Calibrating gyro bias... Keep MPU6050 still.");

    for (int i = 0; i < samples; i++) {
        sumX += mpu.getRotationX();
        sumY += mpu.getRotationY();
        delay(2);
    }

    gyroBiasX = -sumX / (float)samples;
    gyroBiasY = -sumY / (float)samples;

    Serial.print("Gyro bias X: "); Serial.println(gyroBiasX);
    Serial.print("Gyro bias Y: "); Serial.println(gyroBiasY);
}

void readRawData(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
    ax = mpu.getAccelerationX();
    ay = mpu.getAccelerationY();
    az = mpu.getAccelerationZ();
    gx = mpu.getRotationX();
    gy = mpu.getRotationY();
    gz = mpu.getRotationZ();
}

void computeAngles(float dt, int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy) {
    // Convert to float
    float axf = ax;
    float ayf = ay;
    float azf = az;

    // Accelerometer angle estimates (radians)
    float accelRoll  = atan2(ayf, azf);
    float accelPitch = atan2(-axf, sqrt(ayf * ayf + azf * azf));

    // Gyroscope rate in rad/s (bias corrected)
    float gyroRollRate  = (gx + gyroBiasX) / 131.0 * DEG_TO_RAD;
    float gyroPitchRate = (gy + gyroBiasY) / 131.0 * DEG_TO_RAD;

    // Complementary filter
    roll  = alpha * (roll  + gyroRollRate * dt)  + (1 - alpha) * accelRoll;
    pitch = alpha * (pitch + gyroPitchRate * dt) + (1 - alpha) * accelPitch;
}

void printDebug(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz, float dt) {
    Serial.println("===== DEBUG =====");

    Serial.print("Raw Accel [X,Y,Z] = ");
    Serial.print(ax); Serial.print(", ");
    Serial.print(ay); Serial.print(", ");
    Serial.println(az);

    Serial.print("Raw Gyro [X,Y,Z] = ");
    Serial.print(gx); Serial.print(", ");
    Serial.print(gy); Serial.print(", ");
    Serial.println(gz);

    Serial.print("Δt (s): ");
    Serial.println(dt, 4);

    Serial.print("Filtered Pitch: ");
    Serial.print(pitch, 3);
    Serial.print(" rad (");
    Serial.print(pitch * RAD_TO_DEG, 1);
    Serial.println(" deg)");

    Serial.print("Filtered Roll: ");
    Serial.print(roll, 3);
    Serial.print(" rad (");
    Serial.print(roll * RAD_TO_DEG, 1);
    Serial.println(" deg)");

    Serial.println();
}