#include "fake_arduino/Arduino.h"
#include <iostream>
#include <string>
#include "../BalanceBotLolin32lite/BalanceBotLolin32lite.ino"
#include <cmath>
#include "../test_utils.h"

namespace {

void test_setup_configures_pins_and_logs() {
    fake_arduino::reset();

    setup();

    expect_true(leftSensor.initialized(), "left AS5600 sensor should be initialized");
    expect_true(rightSensor.initialized(), "right AS5600 sensor should be initialized");
    expect_true(leftDriver.initialized(), "left driver should be initialized");
    expect_true(rightDriver.initialized(), "right driver should be initialized");
    expect_true(leftMotor.initialized(), "left motor should be initialized");
    expect_true(rightMotor.initialized(), "right motor should be initialized");
    expect_true(leftMotor.foc_initialized(), "left motor FOC should be initialized");
    expect_true(rightMotor.foc_initialized(), "right motor FOC should be initialized");

    const std::string logs = fake_arduino::serial_log();
    expect_contains(logs, "BalanceBot SimpleFOC boot", "boot banner should be printed");
    expect_contains(logs, "SimpleFOC motors ready.", "ready message should be printed");
}

}  // namespace