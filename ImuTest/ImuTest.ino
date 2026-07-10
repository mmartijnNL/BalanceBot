#include <Arduino.h>
#include <MPU6050_light.h>
#include <Wire.h>
#include <cmath>

namespace {

constexpr unsigned long kTelemetryPeriodMs = 100UL;

// Create Sensors
TwoWire i2cLeft = TwoWire(0);

MPU6050 imu = MPU6050(i2cLeft);


}  // namespace

void setup() {
    Serial.begin(115200);
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
    
    imu.calcOffsets(true, true);

    imu.update();

    Serial.println("Initialized.");
}

void loop() {
    static unsigned long lastTelemetryMs = 0;

    imu.update();

    float angle = atan2(imu.getAccY(), imu.getAccZ()) * (180.0 / PI);

    const unsigned long nowMs = millis();
    if ((nowMs - lastTelemetryMs) >= kTelemetryPeriodMs) {
        Serial.print(angle);
        Serial.print("\t");
        Serial.print(imu.getAccAngleX());   // 1
        Serial.print("\t");
        Serial.print(imu.getAccAngleY());   // 2
        Serial.print("\t");
        Serial.print(imu.getAngleX());      // 3
        Serial.print("\t");
        Serial.print(imu.getAngleY());      // 4
        Serial.print("\t");
        Serial.print(imu.getAngleZ());
        Serial.print("\t");
        Serial.print(imu.getAccX());
        Serial.print("\t");
        Serial.print(imu.getAccXoffset());
        Serial.print("\t");
        Serial.print(imu.getAccY());
        Serial.print("\t");
        Serial.print(imu.getAccYoffset());
        Serial.print("\t");
        Serial.print(imu.getAccZ());
        Serial.print("\t");
        Serial.print(imu.getAccZoffset());
        Serial.print("\t");
        Serial.print(imu.getGyroX());
        Serial.print("\t");
        Serial.print(imu.getGyroY());
        Serial.print("\t");
        Serial.print(imu.getGyroZ());
        Serial.print("\t");
        Serial.print(imu.getTemp());
        Serial.print("\t");
   
        Serial.print("\n");  
        lastTelemetryMs = nowMs;
    }
}
