import os


import matplotlib.pyplot as plt
import pandas as pd
import numpy as np


def main() -> None:
    # Resolve paths whether run from repo root or simulation folder.
    base_dir = os.path.dirname(__file__)
    csv_path = os.path.join(base_dir, "sim_stabilize.csv")
    if not os.path.exists(csv_path):
        csv_path = "sim_stabilize.csv"

    if not os.path.exists(csv_path):
        raise FileNotFoundError("sim_stabilize.csv not found. Run ./sim first.")

    df = pd.read_csv(csv_path)

    # Only require t and pitch_deg for minimal plotting
    required_cols = {"t", "pitch_deg"}
    missing = sorted(required_cols - set(df.columns))
    if missing:
        raise ValueError(f"CSV is missing expected columns: {missing}")

    t = df["t"]
    # Add extra subplot if PID columns are present
    pid_cols = [c for c in ["kp_angle", "ki_angle", "kd_angle"] if c in df.columns]
    n_plots = 5 + (1 if pid_cols else 0)
    fig, axes = plt.subplots(n_plots, 1, figsize=(14, 3*n_plots+1), sharex=True)
    axes = axes if isinstance(axes, (list, np.ndarray)) else [axes]


    # Plotting logic for each available group (skip if missing)
    idx = 0
    if "pitch_deg" in df:
        axes[idx].plot(t, df["pitch_deg"], label="Pitch true (deg)", linewidth=1.8)
        if "pitch_meas_deg" in df:
            axes[idx].plot(t, df["pitch_meas_deg"], label="Pitch measured (deg)", alpha=0.75)
        if "target_angle_deg" in df:
            axes[idx].plot(t, df["target_angle_deg"], label="Target angle (deg)", linestyle="--")
        axes[idx].set_ylabel("Angle")
        axes[idx].set_title("BalanceBot Full-System Simulation")
        axes[idx].grid(True, alpha=0.3)
        axes[idx].legend(loc="upper right")
    idx += 1
    if "gyro_dps" in df:
        axes[idx].plot(t, df["gyro_dps"], label="Gyro true (deg/s)", linewidth=1.8)
        if "gyro_meas_dps" in df:
            axes[idx].plot(t, df["gyro_meas_dps"], label="Gyro measured (deg/s)", alpha=0.75)
        axes[idx].set_ylabel("Rate")
        axes[idx].grid(True, alpha=0.3)
        axes[idx].legend(loc="upper right")
    idx += 1
    if "left_cmd_v" in df:
        axes[idx].plot(t, df["left_cmd_v"], label="Left cmd (V)")
    if "left_actual_v" in df:
        axes[idx].plot(t, df["left_actual_v"], label="Left actual (V)", linestyle="--")
    if "right_cmd_v" in df:
        axes[idx].plot(t, df["right_cmd_v"], label="Right cmd (V)")
    if "right_actual_v" in df:
        axes[idx].plot(t, df["right_actual_v"], label="Right actual (V)", linestyle="--")
    axes[idx].set_ylabel("Motor V")
    axes[idx].grid(True, alpha=0.3)
    axes[idx].legend(loc="upper right", ncol=2)
    idx += 1
    if "batt_v" in df:
        axes[idx].plot(t, df["batt_v"], label="Battery (V)", color="tab:green", linewidth=1.8)
        if "lvc_active" in df:
            lvc_scaled = df["lvc_active"] * (df["batt_v"].max() - df["batt_v"].min()) + df["batt_v"].min()
            axes[idx].plot(t, lvc_scaled, label="LVC active (scaled)", color="tab:red", alpha=0.8)
        axes[idx].set_ylabel("Battery")
        axes[idx].grid(True, alpha=0.3)
        axes[idx].legend(loc="upper right")
    idx += 1
    if "rc_thr" in df:
        axes[idx].plot(t, df["rc_thr"], label="RC throttle")
    if "rc_str" in df:
        axes[idx].plot(t, df["rc_str"], label="RC steer")
    if "steering_cmd" in df:
        axes[idx].plot(t, df["steering_cmd"], label="Steering cmd", linestyle="--")
    axes[idx].set_ylabel("RC / Cmd")
    axes[idx].set_xlabel("Time (s)")
    axes[idx].grid(True, alpha=0.3)
    axes[idx].legend(loc="upper right")
    idx += 1

    # Plot PID values if present
    if pid_cols:
        for col in pid_cols:
            axes[idx].plot(t, df[col], label=col)
        axes[idx].set_ylabel("PID Value")
        axes[idx].set_xlabel("Time (s)")
        axes[idx].set_title("Auto-tuned PID values")
        axes[idx].grid(True, alpha=0.3)
        axes[idx].legend(loc="upper right")

    fig.tight_layout()

    out_png = os.path.join(base_dir, "sim_results_dashboard.png")
    fig.savefig(out_png, dpi=150)
    print(f"Saved plot: {out_png}")

    # Keep interactive behavior for local desktop runs.
    plt.show()


if __name__ == "__main__":
    main()
