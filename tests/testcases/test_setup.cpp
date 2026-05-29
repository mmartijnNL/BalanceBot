#include "fake_arduino/Arduino.h"
#include <iostream>
#include <string>
#include "../BalanceBotPico/BalanceBotPico.ino"
#include <cmath>
#include "../test_utils.h"

namespace {

void test_setup_configures_pins_and_logs() {
    fake_arduino::reset();

    setup();

    expect_true(fake_arduino::get_analog_resolution_bits() == 12, "setup should configure 12-bit ADC");
    expect_true(fake_arduino::get_pin_mode(26) == INPUT, "battery ADC pin should be INPUT");
    expect_true(fake_arduino::get_pin_mode(20) == INPUT_PULLDOWN, "throttle pin should be INPUT_PULLDOWN");
    expect_true(fake_arduino::get_pin_mode(21) == INPUT_PULLDOWN, "steer pin should be INPUT_PULLDOWN");

    const std::string logs = fake_arduino::serial_log();
    expect_contains(logs, "BalanceBot Pico boot", "boot banner should be printed");
    expect_contains(logs, "Pico target ready", "ready message should be printed");
}

}  // namespace