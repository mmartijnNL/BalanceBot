#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

using byte = uint8_t;

constexpr uint8_t INPUT = 0;
constexpr uint8_t OUTPUT = 1;
constexpr uint8_t INPUT_PULLDOWN = 2;

namespace fake_arduino {

struct PinState {
    int mode;
    int digital_value;
    int analog_value;
};

void reset();
void set_millis(uint64_t value);
void advance_millis(uint64_t delta);

void set_analog(int pin, int value);
void set_digital_input(int pin, int value);
int get_pin_mode(int pin);
int get_digital_output(int pin);
int get_analog_resolution_bits();

void set_mpu_begin_status(uint8_t status);
void set_mpu_angle_x(float value);
void set_mpu_gyro_x(float value);
uint8_t get_mpu_begin_status();
float get_mpu_angle_x();
float get_mpu_gyro_x();

std::string serial_log();
void clear_serial_log();

}  // namespace fake_arduino

class FakeSerial {
   public:
    void begin(unsigned long baud);

    size_t print(const char* text);
    size_t print(int value);
    size_t print(float value);

    size_t println(const char* text);
    size_t println(int value);
    size_t println();

    const std::string& buffer() const;

   private:
    std::string buffer_;
};

extern FakeSerial Serial;

void pinMode(int pin, int mode);
void digitalWrite(int pin, int value);
int digitalRead(int pin);
int analogRead(int pin);
void analogReadResolution(int bits);

unsigned long millis();
unsigned long micros();
void delay(unsigned long milliseconds);
