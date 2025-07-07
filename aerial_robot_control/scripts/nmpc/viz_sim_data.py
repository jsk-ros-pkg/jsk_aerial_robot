"""multi_disturb_plot.py
--------------------------------
Read multiple saved NMPC simulation result files (created by
`visualize_nothing_but_save`) from *folder_path* and overlay them on a single
8‑panel disturbance/pose figure—disturbance force, pₓ, pᵧ, p_z, disturbance
torque, roll, pitch, yaw.  All runs that share the same timestamp constitute
one "dataset"; every dataset is drawn with its own colour cycle so their traces
can be compared visually.

Filenames are assumed to follow the scheme
    <YYYYMMDD_HHMMSS>_<control_model>_mdl_<sim_model>_mdl_<array>.npy
where *array* is one of:
    x_sim_all | u_sim_all | est_disturb_f_w_all | est_disturb_tau_g_all

Usage (from CLI):
    python multi_disturb_plot.py \
        --folder ../../../../scripts/nmpc/sim_data \
        --ts_sim 0.01           # sampling period [s] used in the sim

The script requires *tf_conversions* and *scienceplots* (implicitly) to be
available, exactly as in the original visualisation code.
"""

from __future__ import annotations

import argparse
import os
import re
from collections import defaultdict
from pathlib import Path
from typing import Dict, List, Tuple

import matplotlib.pyplot as plt
import numpy as np
from tf_conversions import transformations as tf
import scienceplots  # noqa – activates SciencePlots style in Matplotlib

plt.style.use(["science", "grid"])  # global style
plt.rcParams.update({"font.size": 10})
LEGEND_ALPHA = 0.3

# -----------------------------------------------------------------------------
# Helpers
# -----------------------------------------------------------------------------
# Regex pattern: timestamp, control‑model (ending with _mdl), sim‑model (ending
# with _mdl), then the array name.
_PATTERN = re.compile(
    r"(?P<stamp>\d{8}_\d{6})_"  # e.g. 20250702_162226
    r"(?P<control>.*?_mdl)_"  # up to and incl. first _mdl
    r"(?P<sim>.*?_mdl)_"  # up to and incl. second _mdl
    r"(?P<arr>x_sim_all|u_sim_all|est_disturb_f_w_all|est_disturb_tau_g_all)"  # array type
    r"\.npy"
)


def _scan_folder(folder: Path) -> Dict[str, Dict[str, Path]]:
    """Return dict keyed by timestamp → dict(array_type → Path)."""
    datasets: Dict[str, Dict[str, Path]] = defaultdict(dict)

    for file in folder.glob("*.npy"):
        m = _PATTERN.match(file.name)
        if not m:
            continue
        info = m.groupdict()
        key = info["stamp"]
        # Human‑readable label: control→sim (strip trailing _mdl)
        ctrl_label = info["control"]
        sim_label = info["sim"]
        label = f"{ctrl_label}→{sim_label}"
        datasets[key].setdefault("label", label)
        datasets[key][info["arr"]] = file
    return datasets


def _load_arrays(entry: Dict[str, Path]) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, str]:
    """Load arrays and return (x, u, f_dist, tau_dist, label)."""
    try:
        x = np.load(entry["x_sim_all"], allow_pickle=False)
        u = np.load(entry["u_sim_all"], allow_pickle=False)
        f_dist = np.load(entry["est_disturb_f_w_all"], allow_pickle=False)
        tau_dist = np.load(entry["est_disturb_tau_g_all"], allow_pickle=False)
    except KeyError as exc:
        raise RuntimeError(
            f"Incomplete data set for timestamp {entry.get('label', 'unknown')}"
        ) from exc
    return x, u, f_dist, tau_dist, entry["label"]


def _quaternion_array_to_euler_deg(qwxyz: np.ndarray) -> np.ndarray:
    """Convert Nx4 (qw,qx,qy,qz) → Nx3 Euler (deg) using 3‑2‑1 sequence."""
    euler = np.zeros((qwxyz.shape[0], 3))
    for i, q in enumerate(qwxyz):
        # Convert (qw,qx,qy,qz) → (w,x,y,z) order accepted by tf
        q_xyzw = np.concatenate((q[1:], q[:1]))
        euler[i, :] = tf.euler_from_quaternion(q_xyzw, axes="sxyz")
    return np.rad2deg(euler)


# -----------------------------------------------------------------------------
# Plotting
# -----------------------------------------------------------------------------

def plot_disturb_multi(folder: Path, ts_sim: float) -> None:
    """Overlay all simulation runs in *folder* on a single two‑column figure."""
    entries = _scan_folder(folder)
    if not entries:
        raise FileNotFoundError(f"No .npy files in {folder}")

    # ── Matplotlib defaults ─────────────────────────────────────────────
    plt.rcParams.update({'font.size': 11})
    label_size = 14

    # Create a 7×2 grid of sub‑plots (share x within each row)
    fig, axes = plt.subplots(nrows=7, ncols=2, sharex="row",
                             figsize=(14, 11),
                             # gridspec_kw={"wspace": 0.15, "hspace": 0.25}
                             )

    # Unpack axes for clarity --------------------------------------------------
    # Left column (linear)
    ax_f, ax_px, ax_py, ax_pz, ax_vx, ax_vy, ax_vz = axes[:, 0]
    # Right column (angular)
    ax_tau, ax_roll, ax_pitch, ax_yaw, ax_wx, ax_wy, ax_wz = axes[:, 1]

    # Label y‑axes -------------------------------------------------------------
    ax_f.set_ylabel(r"${^W\boldsymbol{f}_e}$ [N]", fontsize=label_size)
    ax_tau.set_ylabel(r"${^B\boldsymbol{\tau}_e}$ [N$\cdot$m]", fontsize=label_size)

    ax_px.set_ylabel(r"$p_x$ [m]", fontsize=label_size)
    ax_py.set_ylabel(r"$p_y$ [m]", fontsize=label_size)
    ax_pz.set_ylabel(r"$p_z$ [m]", fontsize=label_size)
    ax_vx.set_ylabel(r"$v_x$ [m/s]", fontsize=label_size)
    ax_vy.set_ylabel(r"$v_y$ [m/s]", fontsize=label_size)
    ax_vz.set_ylabel(r"$v_z$ [m/s]", fontsize=label_size)

    ax_roll.set_ylabel(r"Roll [$^\circ$]", fontsize=label_size)
    ax_pitch.set_ylabel(r"Pitch [$^\circ$]", fontsize=label_size)
    ax_yaw.set_ylabel(r"Yaw [$^\circ$]", fontsize=label_size)
    ax_wx.set_ylabel(r"$\omega_x$ [rad/s]", fontsize=label_size)
    ax_wy.set_ylabel(r"$\omega_y$ [rad/s]", fontsize=label_size)
    ax_wz.set_ylabel(r"$\omega_z$ [rad/s]", fontsize=label_size)

    # Disturbance wrench plotted only once (assumed identical across runs)
    drawn_force = drawn_torque = False

    last_t: np.ndarray | None = None  # for x‑axis limits

    line_styles = ['-', '-', ':', ':', ':', '--', '--', '--', '-.']

    # Iterate datasets sorted by timestamp ------------------------------------
    for i, (stamp, files) in enumerate(sorted(entries.items())):
        line_style = line_styles[i % len(line_styles)]

        try:
            x, _u, f_d, tau_d, lbl = _load_arrays(files)
        except RuntimeError as e:
            print(e)
            continue

        t = np.arange(_u.shape[0]) * ts_sim
        x = x[1:]  # drop duplicate first line
        if last_t is None or t[-1] > last_t[-1]:
            last_t = t

        # Left column (linear) -------------------------------------------------
        if not drawn_force:
            ax_f.plot(t, f_d[:, 0], label=r"$f_x$", color="#0072BD")
            ax_f.plot(t, f_d[:, 1], label=r"$f_y$", color="#D95319")
            ax_f.plot(t, f_d[:, 2], label=r"$f_z$", color="#EDB120")
            drawn_force = True

        ax_px.plot(t, x[:, 0], label=lbl, linestyle=line_style)
        ax_py.plot(t, x[:, 1], label=lbl, linestyle=line_style)
        ax_pz.plot(t, x[:, 2], label=lbl, linestyle=line_style)
        ax_vx.plot(t, x[:, 3], label=lbl, linestyle=line_style)
        ax_vy.plot(t, x[:, 4], label=lbl, linestyle=line_style)
        ax_vz.plot(t, x[:, 5], label=lbl, linestyle=line_style)

        # Right column (angular) ----------------------------------------------
        if not drawn_torque:
            ax_tau.plot(t, tau_d[:, 0], label=r"$\tau_x$", color="#0072BD")
            ax_tau.plot(t, tau_d[:, 1], label=r"$\tau_y$", color="#D95319")
            ax_tau.plot(t, tau_d[:, 2], label=r"$\tau_z$", color="#EDB120")
            drawn_torque = True

        euler_deg = _quaternion_array_to_euler_deg(x[:, 6:10])
        ax_roll.plot(t, euler_deg[:, 0], label=lbl, linestyle=line_style)
        ax_pitch.plot(t, euler_deg[:, 1], label=lbl, linestyle=line_style)
        ax_yaw.plot(t, euler_deg[:, 2], label=lbl, linestyle=line_style)
        ax_wx.plot(t, x[:, 10], label=lbl, linestyle=line_style)
        ax_wy.plot(t, x[:, 11], label=lbl, linestyle=line_style)
        ax_wz.plot(t, x[:, 12], label=lbl, linestyle=line_style)

    # Styling & shared tweaks --------------------------------------------------
    for ax in axes.ravel():
        ax.grid(True, alpha=0.3)
        if last_t is not None:
            ax.set_xlim([0.0, last_t[-1]])

    # x‑labels only for bottom row
    axes[-1, 0].set_xlabel("$t$ [s]", fontsize=label_size)
    axes[-1, 1].set_xlabel("$t$ [s]", fontsize=label_size)

    # Local legends for disturbance wrench sub‑plots --------------------------
    ax_f.legend(framealpha=LEGEND_ALPHA, ncol=3, fontsize=label_size)
    ax_tau.legend(framealpha=LEGEND_ALPHA, ncol=3, fontsize=label_size)

    # Global legend for run labels (use last sampled handles)
    handles, _ = axes[-1, 0].get_legend_handles_labels()
    labels = ["Track. NMPC", "Imp. Nominal", "Imp. Nominal ($K=12$)",
              "Imp. Nominal ($D=50$)", "Imp. Nominal ($M=15$)", "Imp. NMPC ($\gamma=1)$", "Imp. NMPC ($\gamma=8$)",
              "Imp. NMPC ($\gamma=16$)", "Imp. NMPC ($\gamma=8,Q_\\alpha=R_f=0$)"]

    fig.legend(handles, labels, loc="lower center", bbox_to_anchor=(0.5, -0.01),
               framealpha=LEGEND_ALPHA, ncol=5, frameon=False)

    # Tidy layout -------------------------------------------------------------
    fig.tight_layout(rect=[0, 0.05, 1, 1])  # leave space for bottom legend
    plt.show()


# -----------------------------------------------------------------------------
# CLI entry point
# -----------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description="Overlay multiple NMPC disturb/pose plots")
    parser.add_argument("--folder", type=str, default="./sim_data/",
                        help="Path to directory containing *_x_sim_all.npy … files")
    parser.add_argument("--ts_sim", type=float, default=0.005,
                        help="Simulation sampling period [s] (default: 0.01)")
    args = parser.parse_args()

    plot_disturb_multi(Path(args.folder).expanduser(), args.ts_sim)


if __name__ == "__main__":
    main()
