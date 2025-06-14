#!/usr/bin/env python3
"""
Draw sensor noise – time‑domain → frequency‑domain utility
---------------------------------------------------------
This script helps you inspect sensor‑noise characteristics so that you can
size an appropriate low‑pass (or other) filter.  **Workflow**

1.  Load the CSV exported from *PlotJuggler* (must contain a column named
    ``__time`` for the absolute timestamp).
2.  Group the log into five logical blocks
      • ESC RPM (×4)
      • squared RPM (rpm² – proxy for thrust, ×4)
      • servo angles (×4)
      • IMU accelerometer (x y z)
      • IMU gyroscope (x y z)
3.  Convert each channel from the time domain to the frequency domain via a
    one‑sided FFT whose amplitude is normalised to the original sine
    magnitude.
4.  Differentiate the gyroscope signal with a 5‑tap FIR differentiator
    :math:`[-1, 8, 0, -8, 1] / 12` (≈ central difference) to highlight high‑rate
    content.
5.  Draw **one single figure** with a 3 × 2 *subplot* grid:

````text
   ┌───────────┬────────────┐
   │   RPM     │   RPM²     │
   ├───────────┼────────────┤
   │  Servo    │  IMU‑acc   │
   ├───────────┼────────────┤
   │ IMU‑gyro  │  d/dt gyro │
   └───────────┴────────────┘
````

Run:
```
python draw_sensor_noise.py <your_log>.csv
```

Author: Li Jinjie – 2025‑06‑14
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import scienceplots  # noqa: F401 – activates the "science" style


# ────────────────────────────────────────────────────────────────────────────────
#                               Helper functions
# ────────────────────────────────────────────────────────────────────────────────


def _sampling_freq(time_s: np.ndarray) -> float:
    """Estimate the **sampling frequency** from a monotonically increasing
    timestamp vector (seconds).  We purposely use the **median** of ∆t to be
    robust against the occasional dropped / duplicated sample."""

    if len(time_s) < 2:
        raise ValueError("Need at least two samples to estimate the sampling frequency.")

    dt = np.median(np.diff(time_s))
    if dt <= 0:
        raise ValueError("Non‑positive time difference detected; check the input file.")

    return 1.0 / dt  # Hz


def _one_sided_fft(data: np.ndarray, fs: float) -> Tuple[np.ndarray, np.ndarray]:
    """Return ``(freq, amplitude)`` for a **real‑valued** signal using the
    *one‑sided* convention (0 … Nyquist).

    Normalisation: for a pure sine of amplitude *A*, the corresponding FFT bin
    shows magnitude *A* (i.e. no additional 1/√N factors)."""

    n = len(data)
    # Remove DC bias so we only observe *dynamic* content.
    data = data - np.mean(data)

    fft_vals = np.fft.rfft(data)  # complex spectrum (half)
    amps = np.abs(fft_vals) * 2.0 / n  # convert to amplitude spectrum
    freqs = np.fft.rfftfreq(n, d=1.0 / fs)  # associated frequency bins

    return freqs, amps


def _fft_dataframe(df: pd.DataFrame, value_cols: List[str]) -> Tuple[np.ndarray, Dict[str, np.ndarray]]:
    """Compute the FFT for *every column* listed in *value_cols*.

    Returns ``(freq_vector, {col_name: amplitude_spectrum})`` where all spectra
    share the same frequency vector (implicitly enforced by identical length
    and sample‑rate)."""

    time_s = df["__time"].to_numpy()
    fs = _sampling_freq(time_s)

    freqs: np.ndarray | None = None
    spectra: Dict[str, np.ndarray] = {}

    for col in value_cols:
        f, amp = _one_sided_fft(df[col].to_numpy(), fs)
        spectra[col] = amp
        if freqs is None:
            freqs = f  # store the first frequency vector (they are identical)

    assert freqs is not None  # mypy/pyright appeasement
    return freqs, spectra


def _plot_fft(
        ax: plt.Axes,
        freqs: np.ndarray,
        spectra: Dict[str, np.ndarray],
        title: str,
        has_xlabel: bool = True,
        xlabel: str = "Frequency [Hz]",
        has_ylabel: bool = True,
        ylabel: str = "Amplitude",
        legend_alpha: float = 0.6,
        xlim: float | None = None,
) -> None:
    """Draw *all* spectra belonging to the same group on the provided *Axes*."""

    for label, amp in spectra.items():
        ax.plot(freqs, amp, label=label)

    # ax.set_title(title)
    if has_xlabel:
        ax.set_xlabel(xlabel, fontsize=14)
    if has_ylabel:
        ax.set_ylabel(ylabel, fontsize=14)
    if xlim is not None:
        ax.set_xlim([0.0, xlim])
    ax.grid(True, which="both", ls=":", lw=0.5)
    ax.legend(framealpha=legend_alpha, fontsize=12)


# ────────────────────────────────────────────────────────────────────────────────
#                           Main processing pipeline
# ────────────────────────────────────────────────────────────────────────────────


def main(file_path: str) -> None:
    """CLI entry point – orchestrates the full analysis & plot generation."""

    path = Path(file_path)
    if not path.is_file():
        raise FileNotFoundError(path)

    # -----------------------------------------------------------------------
    # 1.  Load CSV **once** and slice into dedicated DataFrames.  We *rename*
    #     topics for prettier legends.
    # -----------------------------------------------------------------------
    data = pd.read_csv(path)

    # Convenience helpers: (topic → short‑name)
    rpm_topics = {
        "/beetle1/esc_telem/esc_telemetry_1/rpm": "$\\hat{\Omega}_1$",
        "/beetle1/esc_telem/esc_telemetry_2/rpm": "$\\hat{\Omega}_2$",
        "/beetle1/esc_telem/esc_telemetry_3/rpm": "$\\hat{\Omega}_3$",
        "/beetle1/esc_telem/esc_telemetry_4/rpm": "$\\hat{\Omega}_4$",
    }

    servo_topics = {
        "/beetle1/joint_states/gimbal1/position": "$\\hat{\\alpha}_1$",
        "/beetle1/joint_states/gimbal2/position": "$\\hat{\\alpha}_2$",
        "/beetle1/joint_states/gimbal3/position": "$\\hat{\\alpha}_3$",
        "/beetle1/joint_states/gimbal4/position": "$\\hat{\\alpha}_4$",
    }

    acc_topics = {
        "/beetle1/imu/acc_data[0]": "$^B\\hat{a}_{\\text{sf},x}$",
        "/beetle1/imu/acc_data[1]": "$^B\\hat{a}_{\\text{sf},y}$",
        "/beetle1/imu/acc_data[2]": "$^B\\hat{a}_{\\text{sf},z}$",
    }

    gyro_topics = {
        "/beetle1/imu/gyro_data[0]": "$^B\\hat{\\omega}_{x}$",
        "/beetle1/imu/gyro_data[1]": "$^B\\hat{\\omega}_{y}$",
        "/beetle1/imu/gyro_data[2]": "$^B\\hat{\\omega}_{z}$",
    }

    # Drop *any* row with NaN to avoid unequal lengths afterwards.
    data_rpm = data[["__time", *rpm_topics]].dropna().rename(columns=rpm_topics)
    data_servo = data[["__time", *servo_topics]].dropna().rename(columns=servo_topics)
    data_acc = data[["__time", *acc_topics]].dropna().rename(columns=acc_topics)
    data_gyro = data[["__time", *gyro_topics]].dropna().rename(columns=gyro_topics)

    # -----------------------------------------------------------------------
    # 2.  Compute *squared* RPM (“thrust proxy”).  Done in its own DataFrame so
    #     we keep the original intact.
    # -----------------------------------------------------------------------
    data_rpm2 = data_rpm.copy()
    for c in data_rpm2.columns:
        if c != "__time":
            data_rpm2[c] = data_rpm2[c] ** 2

    # -----------------------------------------------------------------------
    # 3.  FFT for each block.  We *store* the result so we can later choose how
    #     to visualise (either separate figures or one grid).
    # -----------------------------------------------------------------------
    plt.style.use(["science", "grid"])
    plt.rcParams.update({"font.size": 11})

    freqs_rpm, spec_rpm = _fft_dataframe(data_rpm, list(rpm_topics.values()))
    freqs_rpm2, spec_rpm2 = _fft_dataframe(data_rpm2, list(rpm_topics.values()))

    # modify the legend labels to indicate squared RPM
    spec_rpm2["$\\hat{\\Omega}^2_1$"] = spec_rpm2["$\\hat{\\Omega}_1$"]  # keep the first channel as a reference
    spec_rpm2["$\\hat{\\Omega}^2_2$"] = spec_rpm2["$\\hat{\\Omega}_2$"]
    spec_rpm2["$\\hat{\\Omega}^2_3$"] = spec_rpm2["$\\hat{\\Omega}_3$"]
    spec_rpm2["$\\hat{\\Omega}^2_4$"] = spec_rpm2["$\\hat{\\Omega}_4$"]
    del spec_rpm2["$\\hat{\\Omega}_1$"]  # remove the original channel
    del spec_rpm2["$\\hat{\\Omega}_2$"]
    del spec_rpm2["$\\hat{\\Omega}_3$"]
    del spec_rpm2["$\\hat{\\Omega}_4$"]

    freqs_servo, spec_servo = _fft_dataframe(data_servo, list(servo_topics.values()))
    freqs_acc, spec_acc = _fft_dataframe(data_acc, list(acc_topics.values()))
    freqs_gyro, spec_gyro = _fft_dataframe(data_gyro, list(gyro_topics.values()))

    # -----------------------------------------------------------------------
    # 4.  FIR differentiator on the gyro: central difference in 5‑point stencil.
    #     We *intentionally* **don’t** use filtfilt (zero‑phase) here because we
    #     later only care about the *amplitude* spectrum.
    # -----------------------------------------------------------------------
    fir_coeff = np.array([-1, 8, 0, -8, 1], dtype=float) / 12.0

    data_gyro_diff = data_gyro.copy()
    for axis in ["$^B\\hat{\\omega}_{x}$", "$^B\\hat{\\omega}_{y}$", "$^B\\hat{\\omega}_{z}$"]:
        data_gyro_diff[axis] = np.convolve(data_gyro[axis].to_numpy(), fir_coeff, mode="same")

    freqs_diff, spec_diff = _fft_dataframe(data_gyro_diff, ["$^B\\hat{\\omega}_{x}$", "$^B\\hat{\\omega}_{y}$",
                                                            "$^B\\hat{\\omega}_{z}$"])

    # modify the legend labels to indicate differentiated gyro
    spec_diff["$^B\\hat{\\dot{\\omega}}_{x}$"] = spec_diff["$^B\\hat{\\omega}_{x}$"]
    spec_diff["$^B\\hat{\\dot{\\omega}}_{y}$"] = spec_diff["$^B\\hat{\\omega}_{y}$"]
    spec_diff["$^B\\hat{\\dot{\\omega}}_{z}$"] = spec_diff["$^B\\hat{\\omega}_{z}$"]
    del spec_diff["$^B\\hat{\\omega}_{x}$"]  # remove the original channel
    del spec_diff["$^B\\hat{\\omega}_{y}$"]
    del spec_diff["$^B\\hat{\\omega}_{z}$"]

    # -----------------------------------------------------------------------
    # 5.  Create a **single figure** with 3 × 2 subplots.
    # -----------------------------------------------------------------------
    fig, axes = plt.subplots(3, 2, figsize=(5, 7), sharex="none")
    axes = axes.flatten()  # easier 1‑D indexing

    _plot_fft(axes[0], freqs_rpm, spec_rpm, "RPM noise spectrum", has_xlabel=False)
    _plot_fft(axes[1], freqs_rpm2, spec_rpm2, "RPM² (thrust proxy) spectrum", has_xlabel=False, has_ylabel=False)
    _plot_fft(axes[2], freqs_servo, spec_servo, "Servo‑angle noise spectrum", has_xlabel=False)
    _plot_fft(axes[3], freqs_acc, spec_acc, "IMU‑accelerometer noise spectrum", has_xlabel=False, has_ylabel=False)
    _plot_fft(axes[4], freqs_gyro, spec_gyro, "IMU‑gyroscope noise spectrum")
    _plot_fft(axes[5], freqs_diff, spec_diff, "Differentiated gyroscope spectrum (FIR)", has_ylabel=False)

    fig.tight_layout()
    # fig.suptitle("Sensor‑noise FFT overview", fontsize=14, y=1.02)

    plt.show()


# ────────────────────────────────────────────────────────────────────────────────
#                                    CLI
# ────────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Convert sensor data to the frequency domain and plot spectra\n"
                    "The CSV should be exported via PlotJuggler with a column named '__time'."
    )
    parser.add_argument("file_path", type=str, help="CSV file containing the logged sensor data")
    args = parser.parse_args()
    main(args.file_path)
