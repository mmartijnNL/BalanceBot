#include <Arduino.h>
#include <MPU6050_light.h>
#include <Wire.h>
#include <cmath>

/// This is a temporary project to test the MPU6050. It should be kept simple.

constexpr unsigned long kTelemetryPeriodMs = 100UL;

// Create Sensors
TwoWire i2cLeft = TwoWire(0);

MPU6050 imu = MPU6050(i2cLeft);

void setup() {
    Serial.begin(460800);
    delay(250);
    Serial.println("\nInitializing");

    // Initialize sensors
    i2cLeft.begin(22, 19, 100000); 

    byte imuStatus = imu.begin();
    while (imuStatus != 0) {
        Serial.print("MPU6050 init failed, status=");
        Serial.println(static_cast<int>(imuStatus));
        imuStatus = imu.begin();
    }
    
    // Calibrate start angle
    imu.calcOffsets(true, true);

    imu.update();

    Serial.println("Initialized.");
}


// Returns an angle in degrees around an axis.
float getAngle() {
    static bool initialized = false;
    static unsigned long lastMicros = 0;
    static float angle = 0.0f;

    const unsigned long nowMicros = micros();
    if (!initialized) {
        initialized = true;
        // Capture startup orientation as the zero reference.
        lastMicros = nowMicros;
        return angle;
    }

    // Gyro output is in deg/s, so integrate over elapsed seconds.
    const float dt = (nowMicros - lastMicros) * 1.0e-6f;
    lastMicros = nowMicros;


    // This gives the right output, but it drifts
    //angle += imu.getGyroX() * dt;

    // This does not drift, but the output is not correct. It ranges from 0 to 64ish. Forward and backward both give the same output of about 45.
    angle = std::atan2(imu.getAccY(), imu.getAccZ()) * 180.0f / PI;

    // Keep output around zero for easier balancing logic.
    while (angle >= 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;

    return angle;
}

void loop() {
    static unsigned long lastTelemetryMs = 0;

    imu.update();

    // Angle is the important value. It should be an angle around an axis.
    float angle = getAngle();

    // Do not change that we print all this.
    const unsigned long nowMs = millis();
    if ((nowMs - lastTelemetryMs) >= kTelemetryPeriodMs) {
        Serial.print("a:");
        Serial.print(angle);
        Serial.print(",acax:");
        Serial.print(imu.getAccAngleX());   // 1
        Serial.print(",acay:");
        Serial.print(imu.getAccAngleY());   // 2
        Serial.print(",ax:");
        Serial.print(imu.getAngleX());      // 3
        Serial.print(",ay:");
        Serial.print(imu.getAngleY());      // 4
        Serial.print(",az:");
        Serial.print(imu.getAngleZ());
        Serial.print(",acx:");
        Serial.print(imu.getAccX());
        Serial.print(",acxo:");
        Serial.print(imu.getAccXoffset());
        Serial.print(",acy:");
        Serial.print(imu.getAccY());
        Serial.print(",acyo:");
        Serial.print(imu.getAccYoffset());
        Serial.print(",acz:");
        Serial.print(imu.getAccZ());
        Serial.print(",aczo:");
        Serial.print(imu.getAccZoffset());
        Serial.print(",gx");
        Serial.print(imu.getGyroX());
        Serial.print(",gy");
        Serial.print(imu.getGyroY());
        Serial.print(",gz");
        Serial.print(imu.getGyroZ());
        Serial.print(",t");
        Serial.print(imu.getTemp());
   
        Serial.print("\n");  
        lastTelemetryMs = nowMs;
    }
}
