#include "Arduino.h"
#include "Wire.h"

#include <algorithm>
#include <array>
#include <cstdio>
#include <sstream>
#include <string>

namespace {

constexpr int kMaxPins = 64;
std::array<fake_arduino::PinState, kMaxPins> g_pins;
std::array<unsigned long, kMaxPins> g_pin_pulse;
uint64_t g_now_ms = 0;
int g_analog_resolution_bits = 10;
uint8_t g_mpu_begin_status = 0;
float g_mpu_angle_x = 0.0f;
float g_mpu_gyro_x = 0.0f;

int clamp_pin(int pin) {
    if (pin < 0) {
        return 0;
    }
    if (pin >= kMaxPins) {
        return kMaxPins - 1;
    }
    return pin;
}

}  // namespace

FakeSerial Serial;
FakeSerial Serial1;
TwoWire Wire(0);
TwoWire Wire1(1);

namespace fake_arduino {

void reset() {
    for (auto& pin : g_pins) {
        pin.mode = INPUT;
        pin.digital_value = 0;
        pin.analog_value = 0;
    }
    g_pin_pulse.fill(0UL);
    g_now_ms = 0;
    g_analog_resolution_bits = 10;
    g_mpu_begin_status = 0;
    g_mpu_angle_x = 0.0f;
    g_mpu_gyro_x = 0.0f;
    clear_serial_log();
    Serial1 = FakeSerial();
}

void set_millis(uint64_t value) {
    g_now_ms = value;
}

void advance_millis(uint64_t delta) {
    g_now_ms += delta;
}

void set_analog(int pin, int value) {
    g_pins[clamp_pin(pin)].analog_value = value;
}

void set_digital_input(int pin, int value) {
    g_pins[clamp_pin(pin)].digital_value = value ? 1 : 0;
}

int get_pin_mode(int pin) {
    return g_pins[clamp_pin(pin)].mode;
}

int get_digital_output(int pin) {
    return g_pins[clamp_pin(pin)].digital_value;
}

int get_analog_resolution_bits() {
    return g_analog_resolution_bits;
}

void set_mpu_begin_status(uint8_t status) {
    g_mpu_begin_status = status;
}

void set_mpu_angle_x(float value) {
    g_mpu_angle_x = value;
}

void set_mpu_gyro_x(float value) {
    g_mpu_gyro_x = value;
}

uint8_t get_mpu_begin_status() {
    return g_mpu_begin_status;
}

float get_mpu_angle_x() {
    return g_mpu_angle_x;
}

float get_mpu_gyro_x() {
    return g_mpu_gyro_x;
}

void set_pin_pulse(int pin, unsigned long value_us) {
    g_pin_pulse[clamp_pin(pin)] = value_us;
}

std::string serial_log() {
    return Serial.buffer();
}

void clear_serial_log() {
    Serial = FakeSerial();
}

}  // namespace fake_arduino

void FakeSerial::begin(unsigned long) {}

size_t FakeSerial::print(const char* text) {
    if (text != nullptr) {
        buffer_ += text;
        return std::char_traits<char>::length(text);
    }
    return 0;
}

size_t FakeSerial::print(int value) {
    std::ostringstream stream;
    stream << value;
    buffer_ += stream.str();
    return stream.str().size();
}

size_t FakeSerial::print(float value) {
    std::ostringstream stream;
    stream << value;
    buffer_ += stream.str();
    return stream.str().size();
}

size_t FakeSerial::println(const char* text) {
    size_t written = print(text);
    buffer_ += "\n";
    return written + 1;
}

size_t FakeSerial::println(int value) {
    size_t written = print(value);
    buffer_ += "\n";
    return written + 1;
}

size_t FakeSerial::println() {
    buffer_ += "\n";
    return 1;
}

const std::string& FakeSerial::buffer() const {
    return buffer_;
}

void pinMode(int pin, int mode) {
    const int idx = clamp_pin(pin);
    g_pins[idx].mode = mode;
    if (mode == INPUT_PULLUP) {
        g_pins[idx].digital_value = HIGH;
    }
}

void digitalWrite(int pin, int value) {
    g_pins[clamp_pin(pin)].digital_value = value ? 1 : 0;
}

int digitalRead(int pin) {
    return g_pins[clamp_pin(pin)].digital_value;
}

int analogRead(int pin) {
    return g_pins[clamp_pin(pin)].analog_value;
}

void analogReadResolution(int bits) {
    g_analog_resolution_bits = bits;
}

unsigned long millis() {
    return static_cast<unsigned long>(g_now_ms);
}

unsigned long micros() {
    return static_cast<unsigned long>(g_now_ms * 1000ULL);
}

void delay(unsigned long milliseconds) {
    g_now_ms += milliseconds;
}

unsigned long pulseIn(uint8_t pin, uint8_t /*state*/, unsigned long /*timeout*/) {
    return g_pin_pulse[clamp_pin(static_cast<int>(pin))];
}
