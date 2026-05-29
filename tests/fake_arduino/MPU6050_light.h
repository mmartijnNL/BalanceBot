#pragma once

#include "Arduino.h"
#include "Wire.h"

class MPU6050 {
   public:
    explicit MPU6050(TwoWire& bus) : bus_(bus) {}

    byte begin() { return fake_arduino::get_mpu_begin_status(); }
    void calcOffsets(bool, bool) {}
    void update() {}

    float getAngleX() { return fake_arduino::get_mpu_angle_x(); }
    float getGyroX() { return fake_arduino::get_mpu_gyro_x(); }

   private:
    TwoWire& bus_;
};
