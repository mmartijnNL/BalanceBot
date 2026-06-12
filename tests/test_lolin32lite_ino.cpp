#include "testcases/test_setup.cpp"
#include "testcases/test_imu.cpp"
#include "testcases/test_motor.cpp"
#include "testcases/test_battery.cpp"
#include "testcases/test_rc.cpp"
#include "../BalanceBotLolin32lite/BalanceBotLolin32lite.ino"

int main() {
    test_setup_configures_pins_and_logs();
    test_imu_init_failure_is_logged();
    test_loop_updates_motor_outputs_from_inputs();
    test_low_battery_triggers_cutoff_message();
    test_rc_steer_shifts_right_motor_velocity();
    test_rc_throttle_shifts_left_torque();
    test_rc_no_signal_is_neutral();

    if (g_failures == 0) {
        std::cout << "All harness tests passed.\n";
        return 0;
    }

    std::cerr << g_failures << " test(s) failed.\n";
    return 1;
}
