#include "fake_arduino/Arduino.h"
#include <iostream>
#include <string>
#include "../BalanceBotPico/BalanceBotPico.ino"
#include <cmath>
#include "../test_utils.h"

namespace {

void test_low_battery_triggers_cutoff_message() {
    fake_arduino::reset();
    setup();

    // Keep battery below cutoff long enough for trip delay.
    fake_arduino::set_analog(26, 0);
    fake_arduino::set_mpu_angle_x(0.0f);
    fake_arduino::set_mpu_gyro_x(0.0f);

    for (int i = 0; i < 140; ++i) {
        fake_arduino::advance_millis(4);
        loop();
    }

    const std::string logs = fake_arduino::serial_log();
    expect_contains(logs, "LOW VOLTAGE CUTOFF ACTIVE", "lvc activation should be printed to Serial");
}

}  // namespace