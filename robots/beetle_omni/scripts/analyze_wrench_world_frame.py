"""
Created by li-jinjie on 25-6-1.
"""
from numexpr.expressions import max_int32

"""
Compute maximum thrust or torque in the world frame for a fully actuated omnidirectional aerial vehicle.
Usage examples:
    python analyze_world_frame.py --mode force --resolution 30
    python analyze_world_frame.py --mode torque --resolution 15
"""

import numpy as np
import cvxpy as cp
import argparse
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from mpl_toolkits.axes_grid1 import make_axes_locatable

from analyze_allocation import get_alloc_mtx_tilt_qd
from analyze_allocation import mass, gravity, p1_b
from analyze_wrench_body_frame import check_wrench_available

# ------------------------------------------------------------------------
# CONSTANTS
# ------------------------------------------------------------------------
THRUST_MAX = 23.0
THRUST_MIN = 0.0
WRENCH_ERROR_LIMIT = 1e-3


# ------------------------------------------------------------------------
def find_max_wrench_for_orientation_world(
    alloc_mtx,
    fg_w,
    roll_deg,
    pitch_deg,
    yaw_deg,
    search_min,
    search_max,
    mode="force",
    tol=1e-2,
    max_iters=30,
    ex_f_w=np.zeros(3),
    ex_tau_w=np.zeros(3),
    install_angle_deg=90,  # Note: for horizontal push, the pitch_deg should be the same w. install angle.
):
    """
    Binary search to find the maximum force or torque along the WORLD z-axis
    for a given orientation, accounting for gravity in the world frame.

    - alloc_mtx: allocation matrix (6×8)
    - fg_w: gravity vector in world frame, e.g., [0, 0, mass*gravity]
    - roll_deg, pitch_deg, yaw_deg: Euler angles (degrees)
    - search_min, search_max: initial search bounds [N] if mode="force", [N·m] if mode="torque"
    - mode: "force" to maximize thrust, "torque" to maximize moment
    - tol: convergence tolerance
    - max_iters: maximum binary search iterations

    Returns:
    - best_value: maximum achievable force [N] if mode="force", torque [N·m] if mode="torque"
    - best_error: wrench error corresponding to best_value
    """
    # Compute rotation from world frame to body frame (R_bw)
    R_bw = R.from_euler("zyx", [yaw_deg, pitch_deg, roll_deg], degrees=True).as_matrix().T
    # Rotate gravity into body frame once
    tgt_f_b = R_bw @ (fg_w + ex_f_w)
    tgt_tau_b = R_bw @ ex_tau_w

    lo, hi = search_min, search_max
    best_value = search_min
    best_error = np.inf

    iter_idx = 0
    for iter_idx in range(max_iters):
        mid = (lo + hi) / 2
        install_angle_rad = np.deg2rad(install_angle_deg)
        mid_b = np.array([mid * np.cos(install_angle_rad), 0.0, mid * np.sin(install_angle_rad)])

        # Build target 6×1 wrench: [Fx; Fy; Fz; τx; τy; τz]
        tgt_wrench = np.zeros((6, 1))
        if mode == "force":
            tgt_wrench[0:3, 0] = tgt_f_b + mid_b
            tgt_wrench[3:6, 0] = tgt_tau_b
        elif mode == "torque":
            tgt_wrench[0:3, 0] = tgt_f_b
            tgt_wrench[3:6, 0] = tgt_tau_b + mid_b
        else:
            raise ValueError("mode must be 'force' or 'torque'")

        ok, err = check_wrench_available(alloc_mtx, tgt_wrench)
        if ok:
            best_value, best_error = mid, err
            lo = mid  # try larger value
        else:
            hi = mid  # reduce value

        if hi - lo < tol:
            break

    if iter_idx == max_iters - 1:
        print(f"Warning: reached maximum iterations ({max_iters})")
        print(f"Final search range: [{lo}, {hi}]")

    return best_value, best_error


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Analyze maximum force or torque in the WORLD frame.")
    parser.add_argument(
        "--mode",
        "-m",
        choices=["force", "torque"],
        required=True,
        help="Select 'force' to compute maximum thrust, or 'torque' to compute maximum torque.",
    )
    parser.add_argument(
        "--resolution",
        "-r",
        type=float,
        default=30,
        help="Resolution of yaw/pitch angles in degrees (e.g., 30 means 30° steps).",
    )
    parser.add_argument("--save_to_npz", action="store_true", help="If set, save the results to a .npz file.")

    args = parser.parse_args()
    mode = args.mode
    resolution = args.resolution
    save_to_npz = args.save_to_npz

    # Load allocation matrix and gravity vector in world
    alloc_mat = get_alloc_mtx_tilt_qd()
    fg_w = np.array([0.0, 0.0, mass * gravity])  # supporting gravity in world frame, so positive z-axis

    plt.style.use(["science", "grid"])
    plt.rcParams.update({"font.size": 14})
    label_size = 15

    if mode == "force":
        thrust_limit = 6 * THRUST_MAX

        install_angle_deg_list = [90, 90, 75, 60, 45, 30, 15, 0, -45, -60]
        pitch_deg_list_list = []
        max_f_result_list = []

        has_plot_45deg = False

        for install_angle_deg in install_angle_deg_list:
            pitch_deg_list = np.linspace(-90 + install_angle_deg, 90 + install_angle_deg, int(180 / resolution) + 1)
            max_f_result = np.zeros(len(pitch_deg_list))

            if not has_plot_45deg:
                yaw_deg = 45
                has_plot_45deg = True
            else:
                yaw_deg = 0

            for i, pitch_deg in enumerate(pitch_deg_list):
                max_f, _ = find_max_wrench_for_orientation_world(
                    alloc_mat,
                    fg_w,
                    roll_deg=0.0,
                    pitch_deg=pitch_deg,
                    yaw_deg=yaw_deg,
                    search_min=0.0,
                    search_max=thrust_limit,
                    mode="force",
                    install_angle_deg=install_angle_deg,
                )

                max_f_result[i] = max_f

            pitch_deg_list_list.append(pitch_deg_list)
            max_f_result_list.append(max_f_result)

            print(f"[World Force] Completed install angle = {install_angle_deg:.1f}°")

        # ---------- 2D X–Z projection plot ----------
        fig, ax = plt.subplots(figsize=(5, 8))

        is_first = True
        for i, install_angle_deg in enumerate(install_angle_deg_list):
            pitch_deg_list = pitch_deg_list_list[i]
            max_f_result = max_f_result_list[i]
            current_lw = 3.0 if install_angle_deg in (0, 90) else 1.5

            if is_first:
                ax.plot(
                    max_f_result * np.cos(np.deg2rad(install_angle_deg - pitch_deg_list)),
                    max_f_result * np.sin(np.deg2rad(install_angle_deg - pitch_deg_list)),
                    label=f"$\\lambda={install_angle_deg}^\\circ$, $+$",
                    linestyle="-",
                    linewidth=1.5,
                    markersize=5,
                )
                is_first = False
                continue

            if install_angle_deg not in [0, -45, -60]:
                ax.plot(
                    max_f_result * np.cos(np.deg2rad(install_angle_deg - pitch_deg_list)),
                    max_f_result * np.sin(np.deg2rad(install_angle_deg - pitch_deg_list)),
                    label=f"$\\lambda={install_angle_deg}^\\circ$, $\\times$",
                    # marker="o",
                    linestyle="-",
                    linewidth=current_lw,
                    markersize=5,
                )
            elif install_angle_deg == 0:
                ax.plot(
                    max_f_result * np.cos(np.deg2rad(install_angle_deg - pitch_deg_list)),
                    max_f_result * np.sin(np.deg2rad(install_angle_deg - pitch_deg_list)),
                    label=f"$\\lambda={install_angle_deg}^\\circ$, $\\times$",
                    linestyle="-.",
                    linewidth=current_lw,
                    markersize=5,
                )
            elif install_angle_deg == -45:
                ax.plot(
                    max_f_result * np.cos(np.deg2rad(install_angle_deg - pitch_deg_list)),
                    max_f_result * np.sin(np.deg2rad(install_angle_deg - pitch_deg_list)),
                    label=f"$\\lambda={install_angle_deg}^\\circ$, $\\times$",
                    # marker="s",
                    color="C4",
                    linestyle="--",
                    linewidth=1.5,
                    markersize=5,
                )
            else:  # install_angle_deg == -60
                ax.plot(
                    max_f_result * np.cos(np.deg2rad(install_angle_deg - pitch_deg_list)),
                    max_f_result * np.sin(np.deg2rad(install_angle_deg - pitch_deg_list)),
                    label=f"$\\lambda={install_angle_deg}^\\circ$, $\\times$",
                    # marker="s",
                    color="C3",
                    linestyle="--",
                    linewidth=1.5,
                    markersize=5,
                )

        ax.set_xlabel("$^W f_x$ [N]", fontsize=label_size)
        ax.set_ylabel("$^W f_z$ [N]", fontsize=label_size)
        ax.set_aspect("equal", adjustable="box")

        # ---------- add polar-like rays from origin ----------
        ray_min_deg, ray_max_deg = -90, 90
        n_rays = int((ray_max_deg - ray_min_deg) / 10 + 1)
        ray_len = None
        if ray_len is None:
            xmin, xmax = ax.get_xlim()
            ymin, ymax = ax.get_ylim()
            ray_len = 1.0 * max(abs(xmin), abs(xmax), abs(ymin), abs(ymax))

        angles_deg = np.linspace(ray_min_deg, ray_max_deg, n_rays)
        angles_rad = np.deg2rad(angles_deg)

        for a_deg, a in zip(angles_deg, angles_rad):
            factor = (ray_max_deg - a_deg) / (ray_max_deg - ray_min_deg) * 0.5 + 0.5
            x_end = factor * ray_len * np.cos(a)
            z_end = factor * ray_len * np.sin(a)
            ax.plot([0, x_end], [0, z_end], color="0.7", linewidth=1.0, linestyle="--", zorder=0)

            ax.text(
                1.03 * x_end,
                1.03 * z_end,
                f"${-int(a_deg)}^\circ$",
                fontsize=label_size - 2,
                color="0.5",
                ha="center",
                va="center",
            )

        plt.legend(fontsize=label_size - 2, framealpha=0.5, loc="center left")

        plt.tight_layout()
        fig.savefig("world_force_xz_projection.pdf", bbox_inches="tight", pad_inches=0.3)
        plt.show()

    elif mode == "torque":
        torque_limit = 6 * THRUST_MAX * np.linalg.norm(p1_b[0:2])

        install_angle_deg_list = [90, 90, 75, 60, 45, 30, 15, 0, -45, -60]
        pitch_deg_list_list = []
        max_tau_result_list = []

        has_plot_45deg = False

        for install_angle_deg in install_angle_deg_list:
            pitch_deg_list = np.linspace(-90 + install_angle_deg, 90 + install_angle_deg, int(180 / resolution) + 1)
            max_tau_result = np.zeros(len(pitch_deg_list))

            if not has_plot_45deg:
                yaw_deg = 45
                has_plot_45deg = True
            else:
                yaw_deg = 0

            for i, pitch_deg in enumerate(pitch_deg_list):
                max_tau, _ = find_max_wrench_for_orientation_world(
                    alloc_mat,
                    fg_w,
                    roll_deg=0.0,
                    pitch_deg=pitch_deg,
                    yaw_deg=yaw_deg,
                    search_min=0.0,
                    search_max=torque_limit,
                    mode="torque",
                    install_angle_deg=install_angle_deg,
                )

                max_tau_result[i] = max_tau

            pitch_deg_list_list.append(pitch_deg_list)
            max_tau_result_list.append(max_tau_result)

            print(f"[World Torque] Completed install angle = {install_angle_deg:.1f}°")

        # ---------- 2D X–Z projection plot ----------
        fig, ax = plt.subplots(figsize=(5, 8))

        is_first = True
        for i, install_angle_deg in enumerate(install_angle_deg_list):
            pitch_deg_list = pitch_deg_list_list[i]
            max_tau_result = max_tau_result_list[i]
            current_lw = 3.0 if install_angle_deg in (0, 90) else 1.5

            if is_first:
                ax.plot(
                    max_tau_result * np.cos(np.deg2rad(install_angle_deg - pitch_deg_list)),
                    max_tau_result * np.sin(np.deg2rad(install_angle_deg - pitch_deg_list)),
                    label=f"$\\lambda={install_angle_deg}^\\circ$, $+$",
                    linestyle="-",
                    linewidth=1.5,
                    markersize=5,
                )
                is_first = False
                continue

            if install_angle_deg not in [0, -45, -60]:
                ax.plot(
                    max_tau_result * np.cos(np.deg2rad(install_angle_deg - pitch_deg_list)),
                    max_tau_result * np.sin(np.deg2rad(install_angle_deg - pitch_deg_list)),
                    label=f"$\\lambda={install_angle_deg}^\\circ$, $\\times$",
                    # marker="o",
                    linestyle="-",
                    linewidth=current_lw,
                    markersize=5,
                )
            elif install_angle_deg == 0:
                ax.plot(
                    max_tau_result * np.cos(np.deg2rad(install_angle_deg - pitch_deg_list)),
                    max_tau_result * np.sin(np.deg2rad(install_angle_deg - pitch_deg_list)),
                    label=f"$\\lambda={install_angle_deg}^\\circ$, $\\times$",
                    linestyle="-.",
                    linewidth=current_lw,
                    markersize=5,
                )
            elif install_angle_deg == -45:
                ax.plot(
                    max_tau_result * np.cos(np.deg2rad(install_angle_deg - pitch_deg_list)),
                    max_tau_result * np.sin(np.deg2rad(install_angle_deg - pitch_deg_list)),
                    label=f"$\\lambda={install_angle_deg}^\\circ$, $\\times$",
                    # marker="s",
                    color="C4",
                    linestyle="--",
                    linewidth=1.5,
                    markersize=5,
                )
            else:  # install_angle_deg == -60
                ax.plot(
                    max_tau_result * np.cos(np.deg2rad(install_angle_deg - pitch_deg_list)),
                    max_tau_result * np.sin(np.deg2rad(install_angle_deg - pitch_deg_list)),
                    label=f"$\\lambda={install_angle_deg}^\\circ$, $\\times$",
                    # marker="s",
                    color="C3",
                    linestyle="--",
                    linewidth=1.5,
                    markersize=5,
                )
        ax.set_xlabel("$^W \\tau_x$ [N·m]", fontsize=label_size)
        ax.set_ylabel("$^W \\tau_z$ [N·m]", fontsize=label_size)
        ax.set_aspect("equal", adjustable="box")

        # ---------- add polar-like rays from origin ----------
        ray_min_deg, ray_max_deg = -90, 90
        n_rays = int((ray_max_deg - ray_min_deg) / 10 + 1)
        ray_len = None
        if ray_len is None:
            xmin, xmax = ax.get_xlim()
            ymin, ymax = ax.get_ylim()
            ray_len = 1.00 * max(abs(xmin), abs(xmax), abs(ymin), abs(ymax))

        angles_deg = np.linspace(ray_min_deg, ray_max_deg, n_rays)
        angles_rad = np.deg2rad(angles_deg)

        for a_deg, a in zip(angles_deg, angles_rad):
            if a_deg < 0:
                factor = (ray_max_deg - a_deg) / (ray_max_deg - ray_min_deg) * 0.5 + 0.5
            else:
                factor = (a_deg - ray_min_deg) / (ray_max_deg - ray_min_deg) * 0.5 + 0.5

            x_end = factor * ray_len * np.cos(a)
            z_end = factor * ray_len * np.sin(a)
            ax.plot([0, x_end], [0, z_end], color="0.7", linewidth=1.0, linestyle="--", zorder=0)

            ax.text(
                1.03 * x_end,
                1.03 * z_end,
                f"${-int(a_deg)}^\circ$",
                fontsize=label_size - 2,
                color="0.5",
                ha="center",
                va="center",
            )

        plt.legend(fontsize=label_size - 2, framealpha=0.5, loc="center left")
        plt.tight_layout()
        fig.savefig("world_torque_xz_projection.pdf", bbox_inches="tight", pad_inches=0.3)
        plt.show()

    else:
        raise ValueError(f"Invalid mode: {mode}. Choose 'force' or 'torque'.")
