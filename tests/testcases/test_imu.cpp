#include "fake_arduino/Arduino.h"
#include <iostream>
#include <string>
#include "../BalanceBotEsp32/BalanceBotEsp32.ino"
#include <cmath>
#include "../test_utils.h"

namespace {

void test_imu_init_failure_is_logged() {
    fake_arduino::reset();
    fake_arduino::set_mpu_begin_status(7);

    setup();

    const std::string logs = fake_arduino::serial_log();
    expect_contains(logs, "MPU6050 init failed: 7", "imu init failure code should be logged");
}

}  // namespace