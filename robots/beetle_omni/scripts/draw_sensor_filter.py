#!/usr/bin/env python3
"""
sensor_filter_plot.py - Time-domain LPF visualiser
==================================================
**What’s new (2025-06-14)**

* Expanded to a **3 × 2 subplot grid** (six panels total)
* Added a *thrust* panel: ft = RPM² × 0.000001 × 0.1283
* Moved *angular acceleration* (dω/dt) to its own panel

Layout
------
````text
╔═══════════════════╦═══════════════════╗
║ 1. RPM            ║ 2. Servo angle   ║
╠═══════════════════╬═══════════════════╣
║ 3. Thrust (ft)    ║ 4. IMU-acc       ║
╠═══════════════════╬═══════════════════╣
║ 5. IMU-gyro       ║ 6. dω/dt (gyro)  ║
╚═══════════════════╩═══════════════════╝
````

Usage
-----
```bash
python sensor_filter_plot.py log.csv
```

Dependencies: ``numpy``, ``pandas``, ``matplotlib``, ``scienceplots`` (no SciPy).
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Dict, Tuple

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import scienceplots  # noqa: F401 - enables the "science" style


# ────────────────────────────────────────────────────────────────────────────────
#                               Helper utilities
# ────────────────────────────────────────────────────────────────────────────────


def sampling_freq(time_s: np.ndarray) -> float:
    """Return the **median** sampling frequency for a monotonic time vector."""
    if len(time_s) < 2:
        return 0.0
    dt = np.median(np.diff(time_s))
    return 1.0 / dt if dt > 0 else 0.0


# ────────────────────────────────────────────────────────────────────────────────
#  Down-sampling - keep *last* sample per 10 ms window to achieve 100 Hz
# ────────────────────────────────────────────────────────────────────────────────


def downsample_to_100hz(df: pd.DataFrame) -> pd.DataFrame:
    """Return a copy reduced to 100 Hz (latest sample per 10 ms bin)."""
    bins = (df["__time"] * 100).astype(int)
    latest = df.groupby(bins).tail(1)
    return latest.sort_values("__time").reset_index(drop=True)


def ensure_100hz(df: pd.DataFrame) -> pd.DataFrame:
    """If faster than 100 Hz, down-sample; otherwise return an unchanged copy."""
    return downsample_to_100hz(df) if sampling_freq(df["__time"].to_numpy()) > 100 + 1e-6 else df.copy()


# ────────────────────────────────────────────────────────────────────────────────
#  Minimal 2‑pole IIR (bi‑quad) – Direct‑form I with **warm‑start**
# ────────────────────────────────────────────────────────────────────────────────

def biquad_iir(b: Tuple[float, float, float], a: Tuple[float, float, float], x: np.ndarray) -> np.ndarray:
    """Apply a 2‑pole IIR whose internal state is *initialised with x₀* so
    the very first output equals the first measurement (y₀ = x₀).  This avoids
    a start‑up dip that would otherwise appear when y₋₁ = y₋₂ = 0."""
    if x.size == 0:
        return np.empty_like(x, dtype=float)

    y = np.zeros_like(x, dtype=float)

    # --- initialise past states so that y₀ = x₀ ---------------------------
    y_prev1 = y_prev2 = x0 = x[0]
    x_prev1 = x_prev2 = x0
    y[0] = x0  # explicit first output

    # --- standard DF‑I recurrence starting from n = 1 ----------------------
    for n in range(1, len(x)):
        xn = x[n]
        yn = (
                b[0] * xn
                + b[1] * x_prev1
                + b[2] * x_prev2
                - a[1] * y_prev1
                - a[2] * y_prev2
        )
        y[n] = yn
        # shift history
        x_prev2, x_prev1 = x_prev1, xn
        y_prev2, y_prev1 = y_prev1, yn

    return y


# 8.84 Hz @ 100 Hz
B884 = (0.0547, 0.1094, 0.0547)
A884 = (1.0, -1.2378, 0.4568)

# 20 Hz @ 100 Hz (for angular acceleration)
B20 = (0.2066, 0.4132, 0.2066)
A20 = (1.0, -0.3695, 0.1958)

# 5-tap central-difference FIR - d/dt
FIR_DIFF = np.array([-1, 8, 0, -8, 1], dtype=float) / 12.0

# RPM→thrust constant
THRUST_COEFF = 0.000001 * 0.1283  # ≈ 1.283e-7


# ────────────────────────────────────────────────────────────────────────────────
#                               Plot helpers
# ────────────────────────────────────────────────────────────────────────────────


def overlay(ax: plt.Axes, t: np.ndarray, raw: Dict[str, np.ndarray], flt: Dict[str, np.ndarray],
            title: str, ylabel: str) -> None:
    """Plot raw (faded) and filtered (solid) curves for each channel."""
    for k in raw:
        ax.plot(t, raw[k], alpha=0.30, label=f"{k} raw")
        ax.plot(t, flt[k], label=f"{k} LPF")
    ax.set_title(title)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel(ylabel)
    ax.grid(True)
    ax.legend(fontsize="x-small", ncol=2)


def overlay_single(ax: plt.Axes, t: np.ndarray, data: Dict[str, np.ndarray], title: str, ylabel: str) -> None:
    """Plot a single-style (no raw/filtered distinction) overlay."""
    for k, v in data.items():
        ax.plot(t, v, label=k)
    ax.set_title(title)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel(ylabel)
    ax.grid(True)
    ax.legend(fontsize="x-small")


# ────────────────────────────────────────────────────────────────────────────────
#                      CSV loading and DataFrame preparation
# ────────────────────────────────────────────────────────────────────────────────


def load_and_prepare(csv: Path):
    df = pd.read_csv(csv)

    rpm_map = {
        "/beetle1/esc_telem/esc_telemetry_1/rpm": "rpm1",
        "/beetle1/esc_telem/esc_telemetry_2/rpm": "rpm2",
        "/beetle1/esc_telem/esc_telemetry_3/rpm": "rpm3",
        "/beetle1/esc_telem/esc_telemetry_4/rpm": "rpm4",
    }

    thrust_cmd_map = {
        "/beetle1/four_axes/command/base_thrust[0]": "$f_{1c}$",
        "/beetle1/four_axes/command/base_thrust[1]": "$f_{2c}$",
        "/beetle1/four_axes/command/base_thrust[2]": "$f_{3c}$",
        "/beetle1/four_axes/command/base_thrust[3]": "$f_{4c}$",
    }

    servo_map = {
        "/beetle1/joint_states/gimbal1/position": "servo1",
        "/beetle1/joint_states/gimbal2/position": "servo2",
        "/beetle1/joint_states/gimbal3/position": "servo3",
        "/beetle1/joint_states/gimbal4/position": "servo4",
    }
    acc_map = {
        "/beetle1/imu/acc_data[0]": "acc_x",
        "/beetle1/imu/acc_data[1]": "acc_y",
        "/beetle1/imu/acc_data[2]": "acc_z",
    }
    gyro_map = {
        "/beetle1/imu/gyro_data[0]": "gyro_x",
        "/beetle1/imu/gyro_data[1]": "gyro_y",
        "/beetle1/imu/gyro_data[2]": "gyro_z",
    }

    data_rpm = ensure_100hz(df[["__time", *rpm_map]].dropna().rename(columns=rpm_map))
    data_thrust_cmd = ensure_100hz(df[["__time", *thrust_cmd_map]].dropna().rename(columns=thrust_cmd_map))
    data_servo = ensure_100hz(df[["__time", *servo_map]].dropna().rename(columns=servo_map))
    data_acc = ensure_100hz(df[["__time", *acc_map]].dropna().rename(columns=acc_map))
    data_gyro = ensure_100hz(df[["__time", *gyro_map]].dropna().rename(columns=gyro_map))

    # Align start-time so 0 s = earliest common timestamp
    t0 = max(d["__time"].iloc[0] for d in (data_rpm, data_servo, data_acc, data_gyro))
    for d in (data_rpm, data_servo, data_acc, data_gyro):
        d["__time"] -= t0

    return data_rpm, data_thrust_cmd, data_servo, data_acc, data_gyro


# ────────────────────────────────────────────────────────────────────────────────
#                                    Main
# ────────────────────────────────────────────────────────────────────────────────


def main():
    parser = argparse.ArgumentParser(description="Plot raw vs. LPF-filtered sensor data (time domain)")
    parser.add_argument("csv", type=str, help="PlotJuggler CSV containing a '__time' column")
    args = parser.parse_args()

    csv_path = Path(args.csv)
    if not csv_path.is_file():
        raise FileNotFoundError(csv_path)

    # Load & prep
    rpm_df, thrust_cmd_df, servo_df, acc_df, gyro_df = load_and_prepare(csv_path)
    acc_df["acc_z"] -= 9.798  # Tokyo gravity in m/s²

    # LPF helpers
    def lpf(df, b, a):
        return {c: biquad_iir(b, a, df[c].to_numpy()) for c in df.columns if c != "__time"}

    rpm_raw = {c: rpm_df[c].to_numpy() for c in rpm_df.columns if c != "__time"}
    thrust_cmd_raw = {c: thrust_cmd_df[c].to_numpy() for c in thrust_cmd_df.columns if c != "__time"}
    servo_raw = {c: servo_df[c].to_numpy() for c in servo_df.columns if c != "__time"}
    acc_raw = {c: acc_df[c].to_numpy() for c in acc_df.columns if c != "__time"}
    gyro_raw = {c: gyro_df[c].to_numpy() for c in gyro_df.columns if c != "__time"}

    rpm_lpf = lpf(rpm_df, B884, A884)
    thrust_cmd_lpf = lpf(thrust_cmd_df, B884, A884)
    servo_lpf = lpf(servo_df, B884, A884)
    acc_lpf = lpf(acc_df, B884, A884)
    gyro_lpf = lpf(gyro_df, B884, A884)

    # Thrust (derived from RPM)
    thrust_raw = {k: (rpm_raw[k] ** 2) * THRUST_COEFF for k in rpm_raw}
    thrust_lpf = {k: (rpm_lpf[k] ** 2) * THRUST_COEFF for k in rpm_lpf}

    # Angular acceleration (dω/dt)
    diff_raw = {}
    diff_lpf = {}
    for axis in ("gyro_x", "gyro_y", "gyro_z"):
        d_raw = np.convolve(gyro_df[axis].to_numpy(), FIR_DIFF, mode="same")
        diff_raw[axis.replace("gyro", " $\\alpha$")] = d_raw
        diff_lpf[axis.replace("gyro", " $\\alpha$")] = biquad_iir(B20, A20, d_raw)

    # Plot ------------------------------------------------------------------
    plt.style.use(["science", "grid"])
    plt.rcParams.update({"font.size": 11})

    fig, axes = plt.subplots(3, 2, figsize=(13, 13))
    ax = axes.flatten()

    # overlay(ax[0], rpm_df["__time"], rpm_raw, rpm_lpf, "ESC RPM", "RPM")
    overlay(ax[0], thrust_cmd_df["__time"], thrust_cmd_raw, thrust_cmd_lpf, "Thrust command", "Force [N]")
    overlay(ax[1], rpm_df["__time"], thrust_raw, thrust_lpf, "Prop thrust", "Force [N]")
    overlay(ax[2], servo_df["__time"], servo_raw, servo_lpf, "Servo angle", "Angle [rad]")
    overlay(ax[3], acc_df["__time"], acc_raw, acc_lpf, "IMU accelerometer", "Accel [$m\,s^{-2}$]")
    overlay(ax[4], gyro_df["__time"], gyro_raw, gyro_lpf, "IMU gyro", "Gyro [$rad\,s^{-1}$]")
    overlay(ax[5], gyro_df["__time"], diff_raw, diff_lpf, "Angular acceleration", "$\\alpha$ [$rad\,s^{-2}$]")

    fig.tight_layout()
    fig.suptitle("Sensor signals - raw vs. filtered", fontsize=14, y=1.02)
    plt.show()


if __name__ == "__main__":
    main()
