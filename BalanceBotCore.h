#pragma once
#include <stdint.h>

#ifdef ARDUINO
#include <Arduino.h>
#else
#include <cmath>
#define constrain(x, a, b) (std::max((a), std::min((x), (b))))
#define fabsf std::fabs
#endif

struct BalanceBotConfig {
    float kp_angle = 22.0f;
    float ki_angle = 0.0f;
    float kd_angle = 0.7f;
    float kp_rate = 0.12f;
    float ki_rate = 0.8f;
    float kd_rate = 0.0008f;
    float driver_voltage_limit = 6.0f;
    float max_tilt_deg = 25.0f;
};

struct BalanceBotState {
    float angleInt = 0.0f;
    float prevAngleErr = 0.0f;
    float rateInt = 0.0f;
    float prevRateErr = 0.0f;
};

inline void resetBalanceBotState(BalanceBotState &state) {
    state.angleInt = 0.0f;
    state.prevAngleErr = 0.0f;
    state.rateInt = 0.0f;
    state.prevRateErr = 0.0f;
}

inline float balanceBotControl(
    BalanceBotConfig &cfg,
    BalanceBotState &state,
    float pitchDeg,
    float gyroPitchRateDegS,
    float dt,
    float commandedAngleDeg,
    float commandedSteering,
    float &leftCmd,
    float &rightCmd
) {
    float angleErr = commandedAngleDeg - pitchDeg;
    float dAngle = (angleErr - state.prevAngleErr) / dt;
    state.angleInt += angleErr * dt;
    state.angleInt = constrain(state.angleInt, -15.0f, 15.0f);
    state.prevAngleErr = angleErr;
    float targetRateDegS = cfg.kp_angle * angleErr + cfg.ki_angle * state.angleInt + cfg.kd_angle * dAngle;
    float rateErr = targetRateDegS - gyroPitchRateDegS;
    float dRate = (rateErr - state.prevRateErr) / dt;
    state.rateInt += rateErr * dt;
    state.rateInt = constrain(state.rateInt, -40.0f, 40.0f);
    state.prevRateErr = rateErr;
    float baseCmd = cfg.kp_rate * rateErr + cfg.ki_rate * state.rateInt + cfg.kd_rate * dRate;
    baseCmd = constrain(baseCmd, -cfg.driver_voltage_limit, cfg.driver_voltage_limit);
    leftCmd = baseCmd - commandedSteering;
    rightCmd = baseCmd + commandedSteering;
    return baseCmd;
}
