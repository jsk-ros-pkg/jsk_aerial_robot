'''
 Created by li-jinjie on 25-6-1.
'''
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

# ------------------------------------------------------------------------
# CONSTANTS
# ------------------------------------------------------------------------
THRUST_MAX = 30.0
THRUST_MIN = 0.0
WRENCH_ERROR_LIMIT = 1e-3


# ------------------------------------------------------------------------


def check_wrench_available(alloc_mtx, tgt_wrench, f_th_max=THRUST_MAX, wrench_error_limit=WRENCH_ERROR_LIMIT):
    """
    Determine whether the target wrench can be produced given:
      - alloc_mtx: allocation matrix (6×8)
      - tgt_wrench: desired wrench vector (6×1)
      - f_th_max: maximum thrust magnitude per rotor
      - wrench_error_limit: tolerated wrench error
    Returns:
      - (is_available: bool, wrench_error: float)
    """
    x = cp.Variable(8)
    objective = cp.Minimize(cp.sum_squares(alloc_mtx @ x - tgt_wrench[:, 0]))

    constraints = []
    for i in range(4):
        constraints.append(x[2 * i] ** 2 + x[2 * i + 1] ** 2 <= f_th_max ** 2)

    prob = cp.Problem(objective, constraints)
    try:
        prob.solve(verbose=False)
    except cp.SolverError as e:
        print(f"Solver error when checking wrench availability: {e}")
        return False, np.inf

    if prob.status not in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
        return False, np.inf

    wrench_error = np.linalg.norm(alloc_mtx @ x.value - tgt_wrench[:, 0])
    return (wrench_error <= wrench_error_limit), wrench_error


def find_max_wrench_for_orientation_world(alloc_mtx, fg_w, roll_deg, pitch_deg, yaw_deg,
                                          search_min, search_max,
                                          mode="force", tol=1e-2, max_iters=30):
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
    fg_b = R_bw @ fg_w

    lo, hi = search_min, search_max
    best_value = search_min
    best_error = np.inf

    iter_idx = 0
    for iter_idx in range(max_iters):
        mid = (lo + hi) / 2
        mid_b = np.array([0.0, 0.0, mid])  # the end-effector is installed along the body z-axis

        # Build target 6×1 wrench: [Fx; Fy; Fz; τx; τy; τz]
        tgt_wrench = np.zeros((6, 1))
        if mode == "force":
            tgt_wrench[0:3, 0] = fg_b + mid_b
        elif mode == "torque":
            tgt_wrench[0:3, 0] = fg_b
            tgt_wrench[3:6, 0] = mid_b
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
    parser = argparse.ArgumentParser(
        description="Analyze maximum force or torque in the WORLD frame."
    )
    parser.add_argument(
        "--mode",
        "-m",
        choices=["force", "torque"],
        required=True,
        help="Select 'force' to compute maximum thrust, or 'torque' to compute maximum torque."
    )
    parser.add_argument(
        "--resolution", "-r",
        type=float,
        default=30,
        help="Resolution of yaw/pitch angles in degrees (e.g., 30 means 30° steps)."
    )
    parser.add_argument(
        "--save_to_npz",
        action="store_true",
        help="If set, save the results to a .npz file."
    )

    args = parser.parse_args()
    mode = args.mode
    resolution = args.resolution
    save_to_npz = args.save_to_npz

    # Load allocation matrix and gravity vector in world
    alloc_mat = get_alloc_mtx_tilt_qd()
    fg_w = np.array([0.0, 0.0, mass * gravity])  # supporting gravity in world frame, so positive z-axis

    yaw_list = np.linspace(-180, 180, int(360 / resolution) + 1)
    pitch_list = np.linspace(0, 180, int(180 / resolution) + 1)

    Nyaw = len(yaw_list)
    Npitch = len(pitch_list)

    if mode == "force":
        result_map = np.zeros((Nyaw, Npitch))
        thrust_limit = 6 * THRUST_MAX

        # The result should be the same around the yaw axis
        for j, pitch_deg in enumerate(pitch_list):
            max_f, _ = find_max_wrench_for_orientation_world(
                alloc_mat, fg_w,
                roll_deg=0.0,
                pitch_deg=pitch_deg,
                yaw_deg=0.0,
                search_min=0.0,
                search_max=thrust_limit,
                mode="force"
            )
            result_map[:, j] = max_f
            print(f"[World Force] Completed pitch = {pitch_deg:.1f}°")

        if save_to_npz:
            np.savez("world_force_map.npz", yaw_list=yaw_list, pitch_list=pitch_list, force_map=result_map)
            print("Saved world-frame force map to 'world_force_map.npz'.")

        # ---------- Plot 3D surface in world frame ----------
        dirs = []
        mags = []
        for i, yaw_deg in enumerate(yaw_list):
            for j, pitch_deg in enumerate(pitch_list):
                # Body z-axis expressed in world using zero roll
                R_wb_zero_roll = R.from_euler('zyx', [yaw_deg, pitch_deg, 0.0], degrees=True).as_matrix().T
                dir_z = R_wb_zero_roll[:, 2]
                dirs.append(dir_z)
                mags.append(result_map[i, j])
        dirs = np.array(dirs)
        mags = np.array(mags)

        origins = np.zeros_like(dirs)
        U = dirs[:, 0] * mags
        V = dirs[:, 1] * mags
        W = dirs[:, 2] * mags

        endpoints = origins + np.column_stack((U, V, W))
        X = endpoints[:, 0].reshape(Nyaw, Npitch)
        Y = endpoints[:, 1].reshape(Nyaw, Npitch)
        Z = endpoints[:, 2].reshape(Nyaw, Npitch)

        norm = plt.Normalize(result_map.min(), result_map.max())
        colors = cm.viridis(norm(result_map))

        # Create 3D surface plot for force
        plt.style.use(["science"])

        fig = plt.figure(figsize=(5, 4))
        ax = fig.add_subplot(111, projection="3d")
        surf = ax.plot_surface(
            X, Y, Z,
            facecolors=colors,
            rstride=1, cstride=1,
            linewidth=0, antialiased=False, shade=False
        )
        ax.set_xlabel("$^W f_x$ [N]")
        ax.set_ylabel("$^W f_y$ [N]")
        ax.set_zlabel("$^W f_z$ [N]")
        ax.set_box_aspect((np.ptp(endpoints[:, 0]), np.ptp(endpoints[:, 1]), np.ptp(endpoints[:, 2])))
        fig.savefig("world_force_3d.pdf", bbox_inches="tight", pad_inches=0.3)

        # ---------- 2D X–Z projection scatter plot ----------
        fig2 = plt.figure(figsize=(5, 4))
        ax2 = fig2.add_subplot(111)
        sc2 = ax2.scatter(
            endpoints[:, 0], endpoints[:, 2],
            c=mags, cmap="viridis", s=8
        )
        ax2.set_xlabel("$^W f_x$ [N]")
        ax2.set_ylabel("$^W f_z$ [N]")
        ax2.set_aspect("equal", adjustable="box")
        ax2.grid(True)
        divider = make_axes_locatable(ax2)
        cax = divider.append_axes("bottom", size="6%", pad=0.5)
        cb = fig2.colorbar(sc2, cax=cax, orientation="horizontal")
        cb.set_label("Force [N]")
        plt.tight_layout(rect=[0, 0.0, 1.0, 1.1])
        fig2.savefig("world_force_xz_projection.pdf", bbox_inches="tight", pad_inches=0.3)
        plt.show()

    elif mode == "torque":
        result_map = np.zeros((Nyaw, Npitch))
        torque_limit = 6 * THRUST_MAX * np.linalg.norm(p1_b[0:2])

        for j, pitch_deg in enumerate(pitch_list):
            max_tau, _ = find_max_wrench_for_orientation_world(
                alloc_mat, fg_w,
                roll_deg=0.0,
                pitch_deg=pitch_deg,
                yaw_deg=0.0,
                search_min=0.0,
                search_max=torque_limit,
                mode="torque"
            )
            result_map[:, j] = max_tau
            print(f"[World Torque] Completed pitch = {pitch_deg:.1f}°")

        if save_to_npz:
            np.savez("world_torque_map.npz", yaw_list=yaw_list, pitch_list=pitch_list, torque_map=result_map)
            print("Saved world-frame torque map to 'world_torque_map.npz'.")

        # ---------- Plot 3D surface in world frame (torque) ----------
        dirs = []
        mags = []
        for i, yaw_deg in enumerate(yaw_list):
            for j, pitch_deg in enumerate(pitch_list):
                R_wb_zero_roll = R.from_euler('zyx', [yaw_deg, pitch_deg, 0.0], degrees=True).as_matrix().T
                dir_z = R_wb_zero_roll[:, 2]
                dirs.append(dir_z)
                mags.append(result_map[i, j])
        dirs = np.array(dirs)
        mags = np.array(mags)

        origins = np.zeros_like(dirs)
        U = dirs[:, 0] * mags
        V = dirs[:, 1] * mags
        W = dirs[:, 2] * mags

        endpoints = origins + np.column_stack((U, V, W))
        X = endpoints[:, 0].reshape(Nyaw, Npitch)
        Y = endpoints[:, 1].reshape(Nyaw, Npitch)
        Z = endpoints[:, 2].reshape(Nyaw, Npitch)

        norm = plt.Normalize(result_map.min(), result_map.max())
        colors = cm.plasma(norm(result_map))

        # Create 3D surface plot for force
        plt.style.use(["science"])

        fig = plt.figure(figsize=(5, 4))
        ax = fig.add_subplot(111, projection="3d")
        surf = ax.plot_surface(
            X, Y, Z,
            facecolors=colors,
            rstride=1, cstride=1,
            linewidth=0, antialiased=False, shade=False
        )
        ax.set_xlabel("$^W \\tau_x$ [N·m]")
        ax.set_ylabel("$^W \\tau_y$ [N·m]")
        ax.set_zlabel("$^W \\tau_z$ [N·m]")
        ax.set_box_aspect((np.ptp(endpoints[:, 0]), np.ptp(endpoints[:, 1]), np.ptp(endpoints[:, 2])))
        fig.savefig("world_torque_3d.pdf", bbox_inches="tight", pad_inches=0.3)

        # ---------- 2D X–Z projection scatter plot for torque ----------
        fig2 = plt.figure(figsize=(5, 4))
        ax2 = fig2.add_subplot(111)
        sc2 = ax2.scatter(
            endpoints[:, 0], endpoints[:, 2],
            c=mags, cmap="plasma", s=8
        )
        ax2.set_xlabel("$^W \\tau_x$ [N·m]")
        ax2.set_ylabel("$^W \\tau_z$ [N·m]")
        ax2.set_aspect("equal", adjustable="box")
        ax2.grid(True)
        divider = make_axes_locatable(ax2)
        cax = divider.append_axes("bottom", size="6%", pad=0.5)
        cb = fig2.colorbar(sc2, cax=cax, orientation="horizontal")
        cb.set_label("Torque [N·m]")
        plt.tight_layout(rect=[0, 0.0, 1.0, 1.1])
        fig2.savefig("world_torque_xz_projection.pdf", bbox_inches="tight", pad_inches=0.3)
        plt.show()

    else:
        raise ValueError(f"Invalid mode: {mode}. Choose 'force' or 'torque'.")
