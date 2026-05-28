#include "BalanceBot.h"
#include <stdarg.h>
#include <stdio.h>

static const struct BalanceBotHAL* g_hal = 0;

void BalanceBot_init(const struct BalanceBotHAL* hal, struct BalanceBotConfig* cfg, struct BalanceBotState* state) {
    g_hal = hal;
    // Set defaults if needed
    if (cfg) {
        cfg->kp_angle = 22.0f;
        cfg->ki_angle = 0.0f;
        cfg->kd_angle = 0.7f;
        cfg->kp_rate = 0.12f;
        cfg->ki_rate = 0.8f;
        cfg->kd_rate = 0.0008f;
        cfg->driver_voltage_limit = 6.0f;
        cfg->max_tilt_deg = 25.0f;
        cfg->control_dt_s = 0.004f;
    }
    if (state) {
        state->angleInt = 0.0f;
        state->prevAngleErr = 0.0f;
        state->rateInt = 0.0f;
        state->prevRateErr = 0.0f;
        state->rcTargetAngleDeg = 0.0f;
        state->rcSteeringCmd = 0.0f;
        state->rcSignalValid = false;
        state->batteryVoltageFiltered = -1.0f;
        state->lvcActive = false;
        state->lvcBelowSinceMs = 0;
        state->balancingEnabled = false;
    }
}

static float constrain(float x, float a, float b) {
    return (x < a) ? a : (x > b) ? b : x;
}

void BalanceBot_update(struct BalanceBotConfig* cfg, struct BalanceBotState* state) {
    // Simulate RC update
    float throttleNorm = g_hal->get_rc_throttle();
    float steerNorm = g_hal->get_rc_steer();
    state->rcTargetAngleDeg = throttleNorm * 6.0f; // RC_MAX_TARGET_ANGLE_DEG
    state->rcSteeringCmd = steerNorm * 1.8f; // RC_MAX_STEER_CMD
    state->rcSignalValid = true; // For simulation, always valid

    // Battery voltage filtering
    float vbatt = g_hal->get_battery_voltage();
    if (state->batteryVoltageFiltered < 0.0f) {
        state->batteryVoltageFiltered = vbatt;
    } else {
        state->batteryVoltageFiltered += 0.08f * (vbatt - state->batteryVoltageFiltered);
    }
    // LVC logic (simplified)
    float lvcCutoff = 4 * 3.30f; // BATTERY_SERIES_CELLS * LVC_CUTOFF_PER_CELL_V
    float lvcRecover = 4 * 3.45f;
    unsigned long nowMs = g_hal->millis();
    if (!state->lvcActive) {
        if (state->batteryVoltageFiltered <= lvcCutoff) {
            if (state->lvcBelowSinceMs == 0) state->lvcBelowSinceMs = nowMs;
            if ((nowMs - state->lvcBelowSinceMs) >= 500) {
                state->lvcActive = true;
                state->balancingEnabled = false;
                g_hal->serial_print("LVC ACTIVE\n");
            }
        } else {
            state->lvcBelowSinceMs = 0;
        }
    } else if (state->batteryVoltageFiltered >= lvcRecover) {
        state->lvcActive = false;
        state->lvcBelowSinceMs = 0;
        g_hal->serial_print("LVC CLEARED\n");
    }

    // Main control logic
    float pitchDeg = g_hal->get_pitch_deg();
    float gyroPitchRateDegS = g_hal->get_gyro_pitch_rate_dps();
    float commandedAngleDeg = state->rcTargetAngleDeg;
    float commandedSteering = state->rcSteeringCmd;
    float angleErr = commandedAngleDeg - pitchDeg;
    float dAngle = (angleErr - state->prevAngleErr) / cfg->control_dt_s;
    state->angleInt += angleErr * cfg->control_dt_s;
    state->angleInt = constrain(state->angleInt, -15.0f, 15.0f);
    state->prevAngleErr = angleErr;
    float targetRateDegS = cfg->kp_angle * angleErr + cfg->ki_angle * state->angleInt + cfg->kd_angle * dAngle;
    float rateErr = targetRateDegS - gyroPitchRateDegS;
    float dRate = (rateErr - state->prevRateErr) / cfg->control_dt_s;
    state->rateInt += rateErr * cfg->control_dt_s;
    state->rateInt = constrain(state->rateInt, -40.0f, 40.0f);
    state->prevRateErr = rateErr;
    float baseCmd = cfg->kp_rate * rateErr + cfg->ki_rate * state->rateInt + cfg->kd_rate * dRate;
    baseCmd = constrain(baseCmd, -cfg->driver_voltage_limit, cfg->driver_voltage_limit);
    float leftCmd = baseCmd - commandedSteering;
    float rightCmd = baseCmd + commandedSteering;
    // Output to motors
    g_hal->motor_left_move(leftCmd);
    g_hal->motor_right_move(rightCmd);
}
