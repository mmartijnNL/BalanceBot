#include "fake_arduino/Arduino.h"
#include <iostream>
#include <string>
#include "../BalanceBotEsp32/BalanceBotEsp32.ino"
#include <cmath>
#include "../test_utils.h"

namespace {

void test_imu_init_failure_is_logged() {
    fake_arduino::reset();

    setup();

    expect_true(leftI2cBus.begun(), "left I2C bus should be started");
    expect_true(rightI2cBus.begun(), "right I2C bus should be started");
    expect_true(leftI2cBus.sda_pin() == 19 && leftI2cBus.scl_pin() == 18, "left I2C pin mapping should match sketch constants");
    expect_true(rightI2cBus.sda_pin() == 23 && rightI2cBus.scl_pin() == 5, "right I2C pin mapping should match sketch constants");
    expect_true(leftMotor.controller == MotionControlType::torque, "left motor should run in torque mode");
    expect_true(rightMotor.controller == MotionControlType::velocity, "right motor should run in velocity mode");
}

}  // namespace