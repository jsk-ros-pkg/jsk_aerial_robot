"""
Created by li-jinjie on 25-6-1.
"""

"""
Analyze coupled wrench capability: force-torque coupling analysis.
This script provides two analysis modes:
  1. force_given_torque: Maximum horizontal force vs required torque
  2. torque_given_force: Maximum torque vs required horizontal force

Both modes account for:
  - Gravity compensation
  - Force/torque requirements
  - Rotor thrust constraints

Usage examples:
    # Mode 1: Maximum force given torque requirement
    python analyze_coupled_wrench.py --mode force_given_torque --samples 30

    # Mode 2: Maximum torque given force requirement
    python analyze_coupled_wrench.py --mode torque_given_force --samples 30

    # With custom axes
    python analyze_coupled_wrench.py -m torque_given_force -n 50 --torque_axis z --force_axis x
"""

import numpy as np
import cvxpy as cp
import argparse
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
import scienceplots
from mpl_toolkits.axes_grid1 import make_axes_locatable

from analyze_allocation import get_alloc_mtx_tilt_qd
from analyze_allocation import mass, gravity, p1_b
from analyze_wrench_body_frame import check_wrench_available
from analyze_wrench_world_frame import find_max_wrench_for_orientation_world

# ------------------------------------------------------------------------
# CONSTANTS
# ------------------------------------------------------------------------
THRUST_MAX = 23.0
THRUST_MIN = 0.0
WRENCH_ERROR_LIMIT = 1e-3

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Analyze coupled wrench: maximum horizontal force vs required torque, or vice versa."
    )
    parser.add_argument(
        "--mode",
        "-m",
        choices=["force_given_torque", "torque_given_force"],
        default="force_given_torque",
        help="Analysis mode: 'force_given_torque' (default) or 'torque_given_force'.",
    )
    parser.add_argument(
        "--samples",
        "-n",
        type=int,
        default=20,
        help="Number of samples to evaluate.",
    )

    args = parser.parse_args()
    mode = args.mode
    num_samples = args.samples

    # Load allocation matrix and gravity vector in world
    alloc_mat = get_alloc_mtx_tilt_qd()
    fg_w = np.array([0.0, 0.0, mass * gravity])  # supporting gravity in world frame

    install_ang_deg_list = [90, 75, 60, 45, 30, 15, 0, -45, -60]  # the angle of end-effector

    # === plot configuration ===
    plt.style.use(["science", "grid"])
    plt.rcParams.update({"font.size": 14})
    label_size = 15
    fig, ax = plt.subplots(figsize=(8, 4))
    linewidth = 1.5
    line_style_list = ["-"]

    if mode == "force_given_torque":
        thrust_limit = 6 * THRUST_MAX

        torque_list = np.linspace(0, 20, num_samples)

        max_f_data_list = []
        for install_ang_deg in install_ang_deg_list:
            max_f_list = []
            for given_torque in torque_list:
                max_f, err = find_max_wrench_for_orientation_world(
                    alloc_mat,
                    fg_w,
                    roll_deg=0.0,
                    pitch_deg=install_ang_deg,
                    yaw_deg=0.0,
                    search_min=0.0,
                    search_max=thrust_limit,
                    mode="force",
                    ex_tau_w=np.array([given_torque, 0, 0]),
                    install_angle_deg=install_ang_deg,
                )
                max_f_list.append(max_f)
            max_f_data_list.append(max_f_list)
            print(f"Finished computing for install angle {install_ang_deg} degrees.")

        max_f_yaw_45_list = []
        for given_torque in torque_list:
            max_f, err = find_max_wrench_for_orientation_world(
                alloc_mat,
                fg_w,
                roll_deg=0.0,
                pitch_deg=90,
                yaw_deg=45.0,
                search_min=0.0,
                search_max=thrust_limit,
                mode="force",
                ex_tau_w=np.array([given_torque, 0, 0]),
                install_angle_deg=90,
            )
            max_f_yaw_45_list.append(max_f)

        # Plotting
        ax.plot(torque_list, np.array(max_f_yaw_45_list), label="$\\lambda=90^\\circ$, + config", linewidth=linewidth)
        for install_ang_deg, max_f_list in zip(install_ang_deg_list, max_f_data_list):
            idx = install_ang_deg_list.index(install_ang_deg)
            current_linewidth = linewidth * 3 if install_ang_deg in (0, 90) else linewidth
            ax.plot(
                torque_list,
                np.array(max_f_list),
                label=f"$\\lambda={int(install_ang_deg)}^\\circ$, $\\times$ config",
                linewidth=current_linewidth,
                linestyle=line_style_list[idx % len(line_style_list)],
            )
        ax.set_xlabel(f"Required Horizontal Torque $^W\\tau_z$ [N·m]", fontsize=label_size)
        ax.set_ylabel(f"Max. Horizontal Force $^Wf_x$ [N]", fontsize=label_size)
        # ax.grid(True, alpha=0.3)
        ax.set_xlim(left=0, right=torque_list[-1])
        ax.set_ylim(bottom=0)

        plt.legend(fontsize=label_size - 2, framealpha=0.85)
        plt.tight_layout()
        plt.show()

    if mode == "torque_given_force":
        thrust_limit = 6 * THRUST_MAX

        force_list = np.linspace(0, 90, num_samples)

        max_tau_data_list = []

        for install_ang_deg in install_ang_deg_list:
            max_tau_list = []
            for given_force in force_list:
                max_tau, err = find_max_wrench_for_orientation_world(
                    alloc_mat,
                    fg_w,
                    roll_deg=0.0,
                    pitch_deg=install_ang_deg,
                    yaw_deg=0.0,
                    search_min=0.0,
                    search_max=thrust_limit,
                    mode="torque",
                    ex_f_w=np.array([given_force, 0, 0]),
                    install_angle_deg=install_ang_deg,
                )
                max_tau_list.append(max_tau)
            max_tau_data_list.append(max_tau_list)
            print(f"Finished computing for install angle {install_ang_deg} degrees.")

        max_tau_yaw_45_list = []
        for given_force in force_list:
            max_tau, err = find_max_wrench_for_orientation_world(
                alloc_mat,
                fg_w,
                roll_deg=0.0,
                pitch_deg=90.0,
                yaw_deg=45.0,
                search_min=0.0,
                search_max=thrust_limit,
                mode="torque",
                ex_f_w=np.array([given_force, 0, 0]),
                install_angle_deg=90.0,
            )
            max_tau_yaw_45_list.append(max_tau)

        # Plotting
        ax.plot(force_list, np.array(max_tau_yaw_45_list), label="$\\lambda=90^\\circ$, $+$", linewidth=linewidth)
        for install_ang_deg, max_tau_list in zip(install_ang_deg_list, max_tau_data_list):
            idx = install_ang_deg_list.index(install_ang_deg)

            if install_ang_deg not in [0, -45, -60]:
                ax.plot(
                    force_list,
                    np.array(max_tau_list),
                    label=f"$\\lambda={int(install_ang_deg)}^\\circ$, $\\times$",
                    linewidth=linewidth * 2 if install_ang_deg == 90 else linewidth,
                    linestyle=line_style_list[idx % len(line_style_list)],
                )
            elif install_ang_deg == 0:
                ax.plot(
                    force_list,
                    np.array(max_tau_list),
                    label=f"$\\lambda={int(install_ang_deg)}^\\circ$, $\\times$",
                    linewidth=linewidth * 2,
                    linestyle="-.",
                )
            elif install_ang_deg == -45:
                ax.plot(
                    force_list,
                    np.array(max_tau_list),
                    label=f"$\\lambda={int(install_ang_deg)}^\\circ$, $\\times$",
                    color="C4",
                    linewidth=linewidth,
                    linestyle="--",
                )
            elif install_ang_deg == -60:
                ax.plot(
                    force_list,
                    np.array(max_tau_list),
                    label=f"$\\lambda={int(install_ang_deg)}^\\circ$, $\\times$",
                    color="C3",
                    linewidth=linewidth,
                    linestyle="--",
                )

        # # fill between for the area under the curve of yaw 45
        # ax.fill_between(
        #     force_list, np.array(max_tau_yaw_45_list), alpha=0.05, color="C0", label="Feasible region", zorder=-1
        # )

        ax.set_xlabel(f"Required Horizontal Force $^Wf_x$ [N]", fontsize=label_size)
        ax.set_ylabel(f"Max. Horizontal Torque $^W\\tau_x$ [N·m]", fontsize=label_size)
        # ax.grid(True, alpha=0.3)
        ax.set_xlim(left=0, right=force_list[-1])
        ax.set_ylim(bottom=0)

        plt.legend(fontsize=label_size - 2, framealpha=0.85, ncol=2)
        plt.tight_layout()
        plt.show()
