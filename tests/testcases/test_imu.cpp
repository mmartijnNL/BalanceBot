#include "fake_arduino/Arduino.h"
#include <iostream>
#include <string>
#include "../BalanceBotLolin32lite/BalanceBotLolin32lite.ino"
#include <cmath>
#include "../test_utils.h"

namespace {

void test_imu_init_failure_is_logged() {
    fake_arduino::reset();
    fake_arduino::set_mpu_begin_status(1);

    setup();

    expect_true(leftI2cBus.begun(), "left I2C bus should be started");
    expect_true(rightI2cBus.begun(), "right I2C bus should be started");
    expect_true(leftI2cBus.sda_pin() == 19 && leftI2cBus.scl_pin() == 18, "left I2C pin mapping should match sketch constants");
    expect_true(rightI2cBus.sda_pin() == 23 && rightI2cBus.scl_pin() == 5, "right I2C pin mapping should match sketch constants");
    expect_true(leftMotor.controller == MotionControlType::torque, "left motor should run in torque mode");
    expect_true(rightMotor.controller == MotionControlType::velocity, "right motor should run in velocity mode");

    const std::string logs = fake_arduino::serial_log();
    expect_contains(logs, "MPU6050 init failed", "imu init failures should be reported to serial logs");
}

}  // namespace