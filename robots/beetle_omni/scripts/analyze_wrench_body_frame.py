"""
Created by li-jinjie on 25-6-1.
"""

"""
Compute maximum thrust or torque in the body frame for a fully actuated omnidirectional aerial vehicle.
Usage examples:
    python analyze_body_frame.py --mode force --resolution 30
    python analyze_body_frame.py --mode torque --resolution 15
"""

import numpy as np
import cvxpy as cp
import argparse
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from matplotlib import cm
import scienceplots
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from mpl_toolkits.axes_grid1 import make_axes_locatable

from analyze_allocation import get_alloc_mtx_tilt_qd
from analyze_allocation import p1_b

# ------------------------------------------------------------------------
# CONSTANTS
# ------------------------------------------------------------------------
THRUST_MAX = 30.0  # Maximum thrust per rotor [N]
THRUST_MIN = 0.0  # Minimum thrust per rotor [N]
WRENCH_ERROR_LIMIT = 1e-3  # Acceptable wrench error


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
    # decision variable: [Fx1, Fy1, Fx2, Fy2, Fx3, Fy3, Fx4, Fy4]
    x = cp.Variable(8)

    # objective: minimize squared error ||A x - tgt_wrench||^2
    objective = cp.Minimize(cp.sum_squares(alloc_mtx @ x - tgt_wrench[:, 0]))

    # constraints: each rotor's (Fx, Fy) magnitude ≤ f_th_max
    constraints = []
    for i in range(4):
        constraints.append(x[2 * i] ** 2 + x[2 * i + 1] ** 2 <= f_th_max**2)

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


def find_max_wrench_for_orientation_body(
    alloc_mtx, roll_deg, pitch_deg, yaw_deg, search_min, search_max, mode="force", tol=1e-2, max_iters=30
):
    """
    Binary search to find the maximum force or torque along the body z-axis
    for a given orientation.

    - alloc_mtx: allocation matrix (6×8)
    - roll_deg, pitch_deg, yaw_deg: Euler angles (degrees)
    - search_min, search_max: initial search bounds [N] if mode="force", [N·m] if mode="torque"
    - mode: "force" to search for thrust, "torque" to search for moment
    - tol: convergence tolerance
    - max_iters: maximum binary search iterations

    Returns:
    - best_value: maximum achievable force [N] if mode="force", torque [N·m] if mode="torque"
    - best_error: wrench error corresponding to best_value
    """
    # Compute rotation from normal frame to body frame (R_bn)
    R_bn = R.from_euler("zyx", [yaw_deg, pitch_deg, roll_deg], degrees=True).as_matrix().T

    lo, hi = search_min, search_max
    best_value = search_min
    best_error = np.inf

    for iter_idx in range(max_iters):
        mid = (lo + hi) / 2
        # mid_n is a vector along the normal frame z-axis
        mid_n = np.array([0.0, 0.0, mid])
        # Transform that vector into body frame
        mid_b = R_bn @ mid_n

        # Build target 6×1 wrench: [Fx; Fy; Fz; τx; τy; τz]
        tgt_wrench = np.zeros((6, 1))
        if mode == "force":
            # assign mid_b to body-frame force components
            tgt_wrench[0:3, 0] = mid_b
        elif mode == "torque":
            # assign mid_b to body-frame torque components
            tgt_wrench[3:6, 0] = mid_b
        else:
            raise ValueError("mode must be 'force' or 'torque'")

        ok, err = check_wrench_available(alloc_mtx, tgt_wrench)
        if ok:
            best_value, best_error = mid, err
            lo = mid  # can try a larger value
        else:
            hi = mid  # reduce value

        if hi - lo < tol:
            break

    if iter_idx == max_iters - 1:
        print(f"Warning: reached maximum iterations ({max_iters})")
        print(f"Final search range: [{lo}, {hi}]")

    return best_value, best_error


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Analyze maximum force or torque in the BODY frame.")
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

    # Load allocation matrix and physical constants
    alloc_mat = get_alloc_mtx_tilt_qd()

    # Create grids of yaw ∈ [-180°, +180°], pitch ∈ [0°, 180°]
    yaw_list = np.linspace(-180, 180, int(360 / resolution) + 1)
    pitch_list = np.linspace(0, 180, int(180 / resolution) + 1)

    Nyaw = len(yaw_list)
    Npitch = len(pitch_list)

    # Allocate map arrays
    if mode == "force":
        result_map = np.zeros((Npitch, Nyaw))
        # Search bounds: [0, 6*THRUST_MAX] — six rotors each at max thrust?
        thrust_limit = 6 * THRUST_MAX

        for i, pitch_deg in enumerate(pitch_list):
            for j, yaw_deg in enumerate(yaw_list):
                max_f, _ = find_max_wrench_for_orientation_body(
                    alloc_mat,
                    roll_deg=0.0,
                    pitch_deg=pitch_deg,
                    yaw_deg=yaw_deg,
                    search_min=0.0,
                    search_max=thrust_limit,
                    mode="force",
                )
                result_map[i, j] = max_f
            print(f"[Body Force] Completed pitch = {pitch_deg:.1f}°")

        # Save to file
        if save_to_npz:
            np.savez("body_force_map.npz", yaw_list=yaw_list, pitch_list=pitch_list, force_map=result_map)
            print("Saved body-frame force map to 'body_force_map.npz'.")

        # ---------- Plotting 3D surface in body frame ----------
        # Flatten directions (body z-axis vectors in world) & magnitudes
        dirs = []
        mags = []
        for i, pitch_deg in enumerate(pitch_list):
            for j, yaw_deg in enumerate(yaw_list):
                R_bn_zero_roll = R.from_euler("zyx", [yaw_deg, pitch_deg, 0.0], degrees=True).as_matrix().T
                dir_z = R_bn_zero_roll[:, 2]  # world ← body: body z-axis expressed in world
                dirs.append(dir_z)
                mags.append(result_map[i, j])
        dirs = np.array(dirs)
        mags = np.array(mags)

        origins = np.zeros_like(dirs)
        U = dirs[:, 0] * mags
        V = dirs[:, 1] * mags
        W = dirs[:, 2] * mags

        # Reshape to (Npitch, Nyaw) grids for surface plotting
        endpoints = origins + np.column_stack((U, V, W))
        X = endpoints[:, 0].reshape(Npitch, Nyaw)
        Y = endpoints[:, 1].reshape(Npitch, Nyaw)
        Z = endpoints[:, 2].reshape(Npitch, Nyaw)

        # Colormap by force magnitude
        norm = plt.Normalize(result_map.min(), result_map.max())
        colors = cm.viridis(norm(result_map))

        # Create 3D surface plot for force
        plt.style.use(["science"])

        fig = plt.figure(figsize=(5, 4))
        ax = fig.add_subplot(111, projection="3d")
        surf = ax.plot_surface(
            X, Y, Z, facecolors=colors, rstride=1, cstride=1, linewidth=0, antialiased=False, shade=False
        )
        ax.set_xlabel("$^B f_x$ [N]")
        ax.set_ylabel("$^B f_y$ [N]")
        ax.set_zlabel("$^B f_z$ [N]")
        ax.set_box_aspect((np.ptp(endpoints[:, 0]), np.ptp(endpoints[:, 1]), np.ptp(endpoints[:, 2])))
        fig.savefig("body_force_3d.pdf", bbox_inches="tight", pad_inches=0.3)

        # ---------- 2D X–Z projection scatter plot ----------
        fig2 = plt.figure(figsize=(5, 4))
        ax2 = fig2.add_subplot(111)
        sc2 = ax2.scatter(endpoints[:, 0], endpoints[:, 2], c=mags, cmap="viridis", s=8)
        ax2.set_xlabel("$^B f_x$ [N]")
        ax2.set_ylabel("$^B f_z$ [N]")
        ax2.set_aspect("equal", adjustable="box")
        ax2.grid(True)
        divider = make_axes_locatable(ax2)
        cax = divider.append_axes("bottom", size="6%", pad=0.5)
        cb = fig2.colorbar(sc2, cax=cax, orientation="horizontal")
        cb.set_label("Force [N]")
        plt.tight_layout(rect=[0.0, 0.0, 1.0, 1.1])
        fig2.savefig("body_force_xz_projection.pdf", bbox_inches="tight", pad_inches=0.3)
        plt.show()

    elif mode == "torque":
        result_map = np.zeros((Npitch, Nyaw))
        # Search bound for torque: 6 * THRUST_MAX * sqrt(p1_b_x^2 + p1_b_y^2)
        torque_limit = 6 * THRUST_MAX * np.linalg.norm(p1_b[0:2])

        for i, pitch_deg in enumerate(pitch_list):
            for j, yaw_deg in enumerate(yaw_list):
                max_tau, _ = find_max_wrench_for_orientation_body(
                    alloc_mat,
                    roll_deg=0.0,
                    pitch_deg=pitch_deg,
                    yaw_deg=yaw_deg,
                    search_min=0.0,
                    search_max=torque_limit,
                    mode="torque",
                )
                result_map[i, j] = max_tau
            print(f"[Body Torque] Completed pitch = {pitch_deg:.1f}°")

        # Save to file
        if save_to_npz:
            np.savez("body_torque_map.npz", yaw_list=yaw_list, pitch_list=pitch_list, torque_map=result_map)
            print("Saved body-frame torque map to 'body_torque_map.npz'.")

        # ---------- Plotting 3D surface in body frame (torque) ----------
        dirs = []
        mags = []
        for i, pitch_deg in enumerate(pitch_list):
            for j, yaw_deg in enumerate(yaw_list):
                R_bn_zero_roll = R.from_euler("zyx", [yaw_deg, pitch_deg, 0.0], degrees=True).as_matrix().T
                dir_z = R_bn_zero_roll[:, 2]
                dirs.append(dir_z)
                mags.append(result_map[i, j])
        dirs = np.array(dirs)
        mags = np.array(mags)

        origins = np.zeros_like(dirs)
        U = dirs[:, 0] * mags
        V = dirs[:, 1] * mags
        W = dirs[:, 2] * mags

        endpoints = origins + np.column_stack((U, V, W))
        X = endpoints[:, 0].reshape(Npitch, Nyaw)
        Y = endpoints[:, 1].reshape(Npitch, Nyaw)
        Z = endpoints[:, 2].reshape(Npitch, Nyaw)

        norm = plt.Normalize(result_map.min(), result_map.max())
        colors = cm.plasma(norm(result_map))

        # Create 3D surface plot for torque
        plt.style.use(["science"])

        fig = plt.figure(figsize=(5, 4))
        ax = fig.add_subplot(111, projection="3d")
        surf = ax.plot_surface(
            X, Y, Z, facecolors=colors, rstride=1, cstride=1, linewidth=0, antialiased=False, shade=False
        )
        ax.set_xlabel("$^B \\tau_x$ [N·m]")
        ax.set_ylabel("$^B \\tau_y$ [N·m]")
        ax.set_zlabel("$^B \\tau_z$ [N·m]")
        ax.set_box_aspect((np.ptp(endpoints[:, 0]), np.ptp(endpoints[:, 1]), np.ptp(endpoints[:, 2])))
        fig.savefig("body_torque_3d.pdf", bbox_inches="tight", pad_inches=0.3)

        # ---------- 2D X–Z projection scatter plot for torque ----------
        fig2 = plt.figure(figsize=(5, 4))
        ax2 = fig2.add_subplot(111)
        sc2 = ax2.scatter(endpoints[:, 0], endpoints[:, 2], c=mags, cmap="plasma", s=8)
        ax2.set_xlabel("$^B \\tau_x$ [N·m]")
        ax2.set_ylabel("$^B \\tau_z$ [N·m]")
        ax2.set_aspect("equal", adjustable="box")
        ax2.grid(True)
        divider = make_axes_locatable(ax2)
        cax = divider.append_axes("bottom", size="6%", pad=0.5)
        cb = fig2.colorbar(sc2, cax=cax, orientation="horizontal")
        cb.set_label("Torque [N·m]")
        plt.tight_layout(rect=[0, 0.0, 1.0, 1.1])
        fig2.savefig("body_torque_xz_projection.pdf", bbox_inches="tight", pad_inches=0.3)
        plt.show()

    else:
        raise ValueError(f"Unknown mode '{mode}'. Use 'force' or 'torque'.")
