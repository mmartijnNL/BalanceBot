#!/usr/bin/env python3
"""BalanceBot controller simulation without hardware.

This script mirrors the control math from BalanceBot.ino and runs it against
a simple inverted-pendulum model so you can evaluate behavior before hardware
is available.
"""

from __future__ import annotations

import argparse
import csv
import math
import random
from dataclasses import dataclass
from typing import Dict, List


@dataclass
class ControllerConfig:
    max_tilt_deg: float = 25.0
    control_dt_s: float = 0.004
    driver_voltage_limit: float = 6.0
    rc_max_target_angle_deg: float = 6.0
    rc_max_steer_cmd: float = 1.8
    kp_angle: float = 22.0
    ki_angle: float = 0.0
    kd_angle: float = 0.7
    kp_rate: float = 0.12
    ki_rate: float = 0.8
    kd_rate: float = 0.0008


@dataclass
class PlantConfig:
    # Positive gravity_gain makes upright position open-loop unstable.
    gravity_gain: float = 1.8
    damping: float = 3.8
    motor_gain: float = 18.0
    sensor_noise_angle_deg: float = 0.05
    sensor_noise_rate_dps: float = 0.2


class BalanceBotController:
    def __init__(self, cfg: ControllerConfig):
        self.cfg = cfg
        self.target_angle_deg = 0.0
        self.steering_cmd = 0.0
        self.balancing_enabled = True

        self.angle_int = 0.0
        self.prev_angle_err = 0.0
        self.rate_int = 0.0
        self.prev_rate_err = 0.0

    def reset_state(self) -> None:
        self.angle_int = 0.0
        self.prev_angle_err = 0.0
        self.rate_int = 0.0
        self.prev_rate_err = 0.0

    def step(
        self,
        pitch_deg: float,
        gyro_pitch_rate_dps: float,
        dt: float,
        rc_target_angle_deg: float,
        rc_steering_cmd: float,
    ) -> Dict[str, float]:
        if (abs(pitch_deg) > self.cfg.max_tilt_deg) or (not self.balancing_enabled):
            self.reset_state()
            return {
                "base_cmd": 0.0,
                "left_cmd": 0.0,
                "right_cmd": 0.0,
                "safety_cut": 1.0,
            }

        commanded_angle_deg = self.target_angle_deg + rc_target_angle_deg
        commanded_steering = self.steering_cmd + rc_steering_cmd

        angle_err = commanded_angle_deg - pitch_deg
        d_angle = (angle_err - self.prev_angle_err) / dt
        self.angle_int += angle_err * dt
        self.angle_int = clamp(self.angle_int, -15.0, 15.0)
        self.prev_angle_err = angle_err

        target_rate_dps = (
            self.cfg.kp_angle * angle_err
            + self.cfg.ki_angle * self.angle_int
            + self.cfg.kd_angle * d_angle
        )

        rate_err = target_rate_dps - gyro_pitch_rate_dps
        d_rate = (rate_err - self.prev_rate_err) / dt
        self.rate_int += rate_err * dt
        self.rate_int = clamp(self.rate_int, -40.0, 40.0)
        self.prev_rate_err = rate_err

        base_cmd = (
            self.cfg.kp_rate * rate_err
            + self.cfg.ki_rate * self.rate_int
            + self.cfg.kd_rate * d_rate
        )
        base_cmd = clamp(base_cmd, -self.cfg.driver_voltage_limit, self.cfg.driver_voltage_limit)

        left_cmd = base_cmd - commanded_steering
        right_cmd = base_cmd + commanded_steering

        return {
            "base_cmd": base_cmd,
            "left_cmd": left_cmd,
            # Mirror firmware orientation (motorRight.move(-rightCmd)).
            "right_cmd": -right_cmd,
            "safety_cut": 0.0,
        }


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def rc_profile(name: str, t: float, cfg: ControllerConfig) -> Dict[str, float]:
    if name == "stabilize":
        throttle_norm = 0.0
        steer_norm = 0.0
    elif name == "rc_step":
        throttle_norm = 0.35 if 1.0 <= t <= 3.0 else 0.0
        steer_norm = 0.30 if 3.2 <= t <= 4.2 else 0.0
    elif name == "disturbance":
        throttle_norm = 0.0
        steer_norm = 0.0
    elif name == "fallover":
        throttle_norm = 0.0
        steer_norm = 0.0
    else:
        raise ValueError(f"Unknown scenario: {name}")

    return {
        "target_angle_deg": throttle_norm * cfg.rc_max_target_angle_deg,
        "steer_cmd": steer_norm * cfg.rc_max_steer_cmd,
    }


def run_simulation(
    duration_s: float,
    dt: float,
    scenario: str,
    seed: int,
    ctrl_cfg: ControllerConfig,
    plant_cfg: PlantConfig,
) -> List[Dict[str, float]]:
    random.seed(seed)

    controller = BalanceBotController(ctrl_cfg)

    if scenario == "fallover":
        theta_deg = 30.0
        theta_dot_dps = 0.0
    else:
        theta_deg = 8.0
        theta_dot_dps = 0.0

    rows: List[Dict[str, float]] = []
    steps = int(duration_s / dt)

    for i in range(steps + 1):
        t = i * dt

        sensed_angle = theta_deg + random.gauss(0.0, plant_cfg.sensor_noise_angle_deg)
        sensed_rate = theta_dot_dps + random.gauss(0.0, plant_cfg.sensor_noise_rate_dps)

        rc = rc_profile(scenario, t, ctrl_cfg)
        cmd = controller.step(
            pitch_deg=sensed_angle,
            gyro_pitch_rate_dps=sensed_rate,
            dt=dt,
            rc_target_angle_deg=rc["target_angle_deg"],
            rc_steering_cmd=rc["steer_cmd"],
        )

        disturbance = 0.0
        if scenario == "disturbance" and 1.5 <= t <= 1.8:
            disturbance = 30.0

        theta_ddot = (
            plant_cfg.gravity_gain * theta_deg
            - plant_cfg.damping * theta_dot_dps
            + plant_cfg.motor_gain * cmd["base_cmd"]
            + disturbance
        )

        theta_dot_dps += theta_ddot * dt
        theta_deg += theta_dot_dps * dt

        # Keep motion in a physically plausible range for the simplified model.
        if theta_deg > 90.0:
            theta_deg = 90.0
            theta_dot_dps = min(theta_dot_dps, 0.0)
        elif theta_deg < -90.0:
            theta_deg = -90.0
            theta_dot_dps = max(theta_dot_dps, 0.0)

        rows.append(
            {
                "t": t,
                "pitch_deg": theta_deg,
                "gyro_dps": theta_dot_dps,
                "target_angle_deg": rc["target_angle_deg"],
                "base_cmd": cmd["base_cmd"],
                "left_cmd": cmd["left_cmd"],
                "right_cmd": cmd["right_cmd"],
                "safety_cut": cmd["safety_cut"],
            }
        )

    return rows


def summarize(rows: List[Dict[str, float]], scenario: str, cfg: ControllerConfig) -> str:
    max_abs_pitch = max(abs(r["pitch_deg"]) for r in rows)
    safety_events = sum(1 for r in rows if r["safety_cut"] > 0.5)

    tail = rows[int(0.8 * len(rows)) :]
    tail_avg_abs_pitch = sum(abs(r["pitch_deg"]) for r in tail) / max(1, len(tail))

    if scenario == "fallover":
        passed = safety_events > 0
        verdict = "PASS" if passed else "FAIL"
        return (
            f"[{verdict}] safety scenario: safety_events={safety_events}, "
            f"max_abs_pitch={max_abs_pitch:.2f} deg"
        )

    settle_threshold = 5.0
    if scenario == "rc_step":
        settle_threshold = 6.0

    passed = tail_avg_abs_pitch < settle_threshold and max_abs_pitch < cfg.max_tilt_deg
    verdict = "PASS" if passed else "FAIL"
    return (
        f"[{verdict}] {scenario}: tail_avg_abs_pitch={tail_avg_abs_pitch:.2f} deg, "
        f"max_abs_pitch={max_abs_pitch:.2f} deg, safety_events={safety_events}"
    )


def write_csv(rows: List[Dict[str, float]], path: str) -> None:
    fieldnames = [
        "t",
        "pitch_deg",
        "gyro_dps",
        "target_angle_deg",
        "base_cmd",
        "left_cmd",
        "right_cmd",
        "safety_cut",
    ]
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    parser = argparse.ArgumentParser(description="BalanceBot controller simulation")
    parser.add_argument(
        "--scenario",
        default="stabilize",
        choices=["stabilize", "rc_step", "disturbance", "fallover"],
        help="Simulation scenario",
    )
    parser.add_argument("--duration", type=float, default=6.0, help="Duration in seconds")
    parser.add_argument("--dt", type=float, default=0.004, help="Control timestep in seconds")
    parser.add_argument("--seed", type=int, default=42, help="Random seed")
    parser.add_argument("--csv", default="", help="Optional CSV output path")
    args = parser.parse_args()

    ctrl_cfg = ControllerConfig(control_dt_s=args.dt)
    plant_cfg = PlantConfig()

    rows = run_simulation(
        duration_s=args.duration,
        dt=args.dt,
        scenario=args.scenario,
        seed=args.seed,
        ctrl_cfg=ctrl_cfg,
        plant_cfg=plant_cfg,
    )

    print(summarize(rows, args.scenario, ctrl_cfg))

    if args.csv:
        write_csv(rows, args.csv)
        print(f"Wrote CSV: {args.csv}")


if __name__ == "__main__":
    main()