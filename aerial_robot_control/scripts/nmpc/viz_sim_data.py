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
    data_entries = _scan_folder(folder)
    if not data_entries:
        raise FileNotFoundError(f"No .npy result files found in {folder}")

    # Prepare figure & sub‑plots
    fig = plt.figure(figsize=(7, 10))
    axes = [plt.subplot(8, 1, i + 1) for i in range(8)]
    ax_force, ax_px, ax_py, ax_pz, ax_torque, ax_roll, ax_pitch, ax_yaw = axes
    ax_roll.set_ylabel(r"Roll [$^\circ$]")
    ax_pitch.set_ylabel(r"Pitch [$^\circ$]")
    ax_yaw.set_ylabel(r"Yaw [$^\circ$]")
    ax_yaw.set_xlabel("Time [s]")

    global t

    if_plot_disturb_wrench = False  # only plot disturb wrench once since all data has the same wrench

    for stamp, entry in sorted(data_entries.items()):
        try:
            x_org, _u, f_d, tau_d, lbl = _load_arrays(entry)
        except RuntimeError as err:
            print(err)
            continue

        t = np.arange(_u.shape[0]) * ts_sim  # Note: The recording frequency of u_cmd is the same as ts_sim
        x = x_org[1:]

        if not if_plot_disturb_wrench:
            # Disturbance force
            ax_force.plot(t, f_d[:, 0], label=rf"$f_x$")
            ax_force.plot(t, f_d[:, 1], label=rf"$f_y$")
            ax_force.plot(t, f_d[:, 2], label=rf"$f_z$")
            # Disturbance torque
            ax_torque.plot(t, tau_d[:, 0], label=rf"$\tau_x$")
            ax_torque.plot(t, tau_d[:, 1], label=rf"$\tau_y$")
            ax_torque.plot(t, tau_d[:, 2], label=rf"$\tau_z$")

            if_plot_disturb_wrench = True

        # Position
        ax_px.plot(t, x[:, 0], label=f"{lbl}")
        ax_py.plot(t, x[:, 1], label=f"{lbl}")
        ax_pz.plot(t, x[:, 2], label=f"{lbl}")
        # Euler angles
        euler_deg = _quaternion_array_to_euler_deg(x[:, 6:10])
        ax_roll.plot(t, euler_deg[:, 0], label=f"{lbl}")
        ax_pitch.plot(t, euler_deg[:, 1], label=f"{lbl}")
        ax_yaw.plot(t, euler_deg[:, 2], label=f"{lbl}")

    # Beautify: labels and limits
    ax_force.set_ylabel(r"Force [N]")
    ax_px.set_ylabel(r"$p_x$ [m]")
    ax_py.set_ylabel(r"$p_y$ [m]")
    ax_pz.set_ylabel(r"$p_z$ [m]")
    ax_torque.set_ylabel(r"Torque [N$\cdot$ m]")

    for ax in axes:
        ax.set_xlim([0.0, t[-1]])
        ax.grid(True, alpha=0.3)

    # # Legends – keep them compact by combining where possible
    ax_force.legend(framealpha=LEGEND_ALPHA, ncol=3, fontsize=7)
    ax_torque.legend(framealpha=LEGEND_ALPHA, ncol=3, fontsize=7)
    # ax_px.legend(framealpha=LEGEND_ALPHA, ncol=2, fontsize=7)
    # # Other axes share same labels; omit duplicate legends to reduce clutter

    fig.subplots_adjust(hspace=0.25)
    plt.tight_layout()
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
