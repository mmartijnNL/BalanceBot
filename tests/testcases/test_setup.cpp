#include "fake_arduino/Arduino.h"
#include <iostream>
#include <string>
#include "../BalanceBotEsp32/BalanceBotEsp32.ino"
#include <cmath>
#include "../test_utils.h"

namespace {

void test_setup_configures_pins_and_logs() {
    fake_arduino::reset();

    setup();

    expect_true(fake_arduino::get_analog_resolution_bits() == 12, "setup should configure 12-bit ADC");
    expect_true(fake_arduino::get_pin_mode(34) == INPUT, "battery ADC pin should be INPUT");
    expect_true(fake_arduino::get_pin_mode(36) == INPUT, "throttle pin should be INPUT");
    expect_true(fake_arduino::get_pin_mode(39) == INPUT, "steer pin should be INPUT");
    expect_true(fake_arduino::get_pin_mode(0) == INPUT_PULLUP, "pause button pin should be INPUT_PULLUP");

    const std::string logs = fake_arduino::serial_log();
    expect_contains(logs, "BalanceBot ESP32 boot", "boot banner should be printed");
    expect_contains(logs, "LOLIN32 Lite target ready", "ready message should be printed");
}

}  // namespace