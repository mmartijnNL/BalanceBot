import os

import matplotlib.pyplot as plt
import pandas as pd


def main() -> None:
    # Resolve paths whether run from repo root or simulation folder.
    base_dir = os.path.dirname(__file__)
    csv_path = os.path.join(base_dir, "sim_stabilize.csv")
    if not os.path.exists(csv_path):
        csv_path = "sim_stabilize.csv"

    if not os.path.exists(csv_path):
        raise FileNotFoundError("sim_stabilize.csv not found. Run ./sim first.")

    df = pd.read_csv(csv_path)

    required_cols = {
        "t",
        "pitch_deg",
        "gyro_dps",
        "pitch_meas_deg",
        "gyro_meas_dps",
        "left_cmd_v",
        "right_cmd_v",
        "left_actual_v",
        "right_actual_v",
        "batt_v",
        "rc_thr",
        "rc_str",
        "target_angle_deg",
        "steering_cmd",
        "lvc_active",
    }
    missing = sorted(required_cols - set(df.columns))
    if missing:
        raise ValueError(f"CSV is missing expected columns: {missing}")

    t = df["t"]
    fig, axes = plt.subplots(5, 1, figsize=(14, 16), sharex=True)

    axes[0].plot(t, df["pitch_deg"], label="Pitch true (deg)", linewidth=1.8)
    axes[0].plot(t, df["pitch_meas_deg"], label="Pitch measured (deg)", alpha=0.75)
    axes[0].plot(t, df["target_angle_deg"], label="Target angle (deg)", linestyle="--")
    axes[0].set_ylabel("Angle")
    axes[0].set_title("BalanceBot Full-System Simulation")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="upper right")

    axes[1].plot(t, df["gyro_dps"], label="Gyro true (deg/s)", linewidth=1.8)
    axes[1].plot(t, df["gyro_meas_dps"], label="Gyro measured (deg/s)", alpha=0.75)
    axes[1].set_ylabel("Rate")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="upper right")

    axes[2].plot(t, df["left_cmd_v"], label="Left cmd (V)")
    axes[2].plot(t, df["left_actual_v"], label="Left actual (V)", linestyle="--")
    axes[2].plot(t, df["right_cmd_v"], label="Right cmd (V)")
    axes[2].plot(t, df["right_actual_v"], label="Right actual (V)", linestyle="--")
    axes[2].set_ylabel("Motor V")
    axes[2].grid(True, alpha=0.3)
    axes[2].legend(loc="upper right", ncol=2)

    axes[3].plot(t, df["batt_v"], label="Battery (V)", color="tab:green", linewidth=1.8)
    lvc_scaled = df["lvc_active"] * (df["batt_v"].max() - df["batt_v"].min()) + df["batt_v"].min()
    axes[3].plot(t, lvc_scaled, label="LVC active (scaled)", color="tab:red", alpha=0.8)
    axes[3].set_ylabel("Battery")
    axes[3].grid(True, alpha=0.3)
    axes[3].legend(loc="upper right")

    axes[4].plot(t, df["rc_thr"], label="RC throttle")
    axes[4].plot(t, df["rc_str"], label="RC steer")
    axes[4].plot(t, df["steering_cmd"], label="Steering cmd", linestyle="--")
    axes[4].set_ylabel("RC / Cmd")
    axes[4].set_xlabel("Time (s)")
    axes[4].grid(True, alpha=0.3)
    axes[4].legend(loc="upper right")

    fig.tight_layout()

    out_png = os.path.join(base_dir, "sim_results_dashboard.png")
    fig.savefig(out_png, dpi=150)
    print(f"Saved plot: {out_png}")

    # Keep interactive behavior for local desktop runs.
    plt.show()


if __name__ == "__main__":
    main()
