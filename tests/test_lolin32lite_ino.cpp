#include "testcases/test_setup.cpp"
#include "testcases/test_imu.cpp"
#include "testcases/test_motor.cpp"
#include "testcases/test_battery.cpp"
#include "../BalanceBotLolin32lite/BalanceBotLolin32lite.ino"

int main() {
    test_setup_configures_pins_and_logs();
    test_imu_init_failure_is_logged();
    test_loop_updates_motor_outputs_from_inputs();
    test_low_battery_triggers_cutoff_message();

    if (g_failures == 0) {
        std::cout << "All harness tests passed.\n";
        return 0;
    }

    std::cerr << g_failures << " test(s) failed.\n";
    return 1;
}
