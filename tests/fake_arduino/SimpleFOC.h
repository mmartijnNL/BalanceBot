#pragma once

#include "Wire.h"

constexpr int AS5600_I2C = 0x36;

enum class MotionControlType {
    torque = 0,
    velocity = 1,
    velocity_openloop = 2,
};

struct PIDController {
    float P = 0.0f;
    float I = 0.0f;
    float D = 0.0f;
};

struct LowPassFilter {
    float Tf = 0.0f;
};

class MagneticSensorI2C {
   public:
    explicit MagneticSensorI2C(int sensor_type) : sensor_type_(sensor_type) {}

    void init(TwoWire* bus) {
        bus_ = bus;
        initialized_ = true;
    }

    bool initialized() const { return initialized_; }
    TwoWire* bus() const { return bus_; }

   private:
    int sensor_type_ = 0;
    bool initialized_ = false;
    TwoWire* bus_ = nullptr;
};

class BLDCDriver3PWM {
   public:
    BLDCDriver3PWM(int pin_u, int pin_v, int pin_w, int pin_enable)
        : pin_u_(pin_u), pin_v_(pin_v), pin_w_(pin_w), pin_enable_(pin_enable) {}

    void init() { initialized_ = true; }

    float voltage_power_supply = 0.0f;
    float voltage_limit = 0.0f;

    bool initialized() const { return initialized_; }

   private:
    int pin_u_ = 0;
    int pin_v_ = 0;
    int pin_w_ = 0;
    int pin_enable_ = 0;
    bool initialized_ = false;
};

class BLDCMotor {
   public:
    explicit BLDCMotor(int pole_pairs) : pole_pairs_(pole_pairs) {}

    void linkSensor(MagneticSensorI2C* sensor) { sensor_ = sensor; }
    void linkDriver(BLDCDriver3PWM* driver) { driver_ = driver; }

    void init() { initialized_ = true; }
    void initFOC() { foc_initialized_ = true; }
    void loopFOC() { loop_count_++; }
    void move(float voltage) { last_move_command = voltage; }
    template <typename T>
    void useMonitoring(T&) {
        monitoring_enabled_ = true;
    }

    float voltage_limit = 0.0f;
    float voltage_sensor_align = 0.0f;
    MotionControlType controller = MotionControlType::torque;
    float last_move_command = 0.0f;
    float shaft_velocity = 0.0f;
    float shaft_angle = 0.0f;
    PIDController PID_velocity;
    LowPassFilter LPF_velocity;

    bool initialized() const { return initialized_; }
    bool foc_initialized() const { return foc_initialized_; }
    bool monitoring_enabled() const { return monitoring_enabled_; }
    int loop_count() const { return loop_count_; }

   private:
    int pole_pairs_ = 0;
    bool initialized_ = false;
    bool foc_initialized_ = false;
    bool monitoring_enabled_ = false;
    int loop_count_ = 0;
    MagneticSensorI2C* sensor_ = nullptr;
    BLDCDriver3PWM* driver_ = nullptr;
};
