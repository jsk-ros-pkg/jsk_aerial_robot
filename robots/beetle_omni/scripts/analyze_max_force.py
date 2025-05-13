'''
 Created by li-jinjie on 25-5-10.
'''
import numpy as np
import cvxpy as cp
import argparse
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from mpl_toolkits.axes_grid1 import make_axes_locatable

from analyze_allocation import get_alloc_mtx_tilt_qd
from analyze_allocation import mass, gravity

# thrust limits
THRUST_MAX = 30.0
THRUST_MIN = 0.0


def check_wrench_available(alloc_mtx, tgt_wrench, f_th_max=THRUST_MAX, wrench_error_limit=1e-3):
    """
    Determine whether the target wrench can be produced given the allocation matrix
    and per-rotor thrust limit f_th_max.
    Returns (is_available, wrench_error).
    """
    # decision variable for rotor forces [Fx1, Fy1, ..., Fx4, Fy4]
    x = cp.Variable(8)

    # objective: minimize squared error between produced and target wrench
    objective = cp.Minimize(cp.sum_squares(alloc_mtx @ x - tgt_wrench[:, 0]))

    # constraints: each rotor force vector magnitude <= max thrust
    constraints = []
    for i in range(4):
        constraints += [x[2 * i] ** 2 + x[2 * i + 1] ** 2 <= f_th_max ** 2]

    # solve the QP
    prob = cp.Problem(objective, constraints)
    try:
        prob.solve(verbose=False)
    except cp.SolverError as e:
        print(f"Solver error: {e}")
        return False, np.inf

    # if solver did not find an optimal solution, treat as infeasible
    if prob.status not in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
        return False, np.inf

    # compute the actual wrench error
    wrench_error = np.linalg.norm(alloc_mtx @ x.value - tgt_wrench[:, 0])
    return (wrench_error <= wrench_error_limit), wrench_error


def find_max_thrust_for_orientation(alloc_mtx, fg_w,
                                    roll_deg, pitch_deg, yaw_deg,
                                    search_thrust_min, search_thrust_max,
                                    is_world,
                                    tol=1e-2, max_iters=30):
    # compute rotation from world to body frame
    R_bw = R.from_euler('zyx', [yaw_deg, pitch_deg, roll_deg], degrees=True).as_matrix().T
    fg_b = R_bw @ fg_w

    lo, hi = search_thrust_min, search_thrust_max
    best_thrust = search_thrust_min
    best_error = np.inf

    iter_idx = 0
    for iter_idx in range(max_iters):
        mid = (lo + hi) / 2

        # target wrench: include gravity plus additional thrust along body z
        if is_world:
            tgt_w = np.array([[fg_b[0], fg_b[1], fg_b[2] + mid, 0.0, 0.0, 0.0]]).T  # the z-axis is thrust
        else:
            mid_b = R_bw @ np.array([0.0, 0.0, mid])
            tgt_w = np.array([[fg_b[0] + mid_b[0], fg_b[0] + mid_b[1], fg_b[0] + mid_b[2], 0.0, 0.0, 0.0]]).T
        ok, err = check_wrench_available(alloc_mtx, tgt_w)
        if ok:
            best_thrust, best_error = mid, err
            lo = mid  # can try larger thrust
        else:
            hi = mid  # reduce thrust
        if hi - lo < tol:
            break

    if iter_idx == max_iters - 1:
        print(f"Warning: max iterations reached ({max_iters}) for thrust search.")
        print(f"Final thrust range: [{lo}, {hi}]")

    return best_thrust, best_error


if __name__ == "__main__":
    # add arguments to choose 1. force or torque 2. the type is world or body
    parser = argparse.ArgumentParser(description="Analyze maximum thrust for a given pitch/yaw orientation.")
    parser.add_argument("--resolution", "-r", type=float, default=30,
                        help="Resolution of pitch/yaw angles in degrees.")
    parser.add_argument(
        "--world", "-w",
        action="store_true",
        help="Use world frame instead of body frame.",
    )

    args = parser.parse_args()
    if_world = args.world

    # init
    alloc_mat = get_alloc_mtx_tilt_qd()

    if args.world:
        fg_w = np.array([0.0, 0.0, mass * gravity])
    else:
        fg_w = np.array([0.0, 0.0, 0.0])  # don't consider gravity in body frame

    # 2) define yaw and pitch grids
    resolution = args.resolution
    yaw_list = np.linspace(-180, 180, int(360 / resolution) + 1)  # from -180° to 180°
    pitch_list = np.linspace(0, 180, int(180 / resolution) + 1)  # from -90° to 90°

    # 3) compute max thrust map
    thrust_map = np.zeros((len(yaw_list), len(pitch_list)))
    for i, yaw_deg in enumerate(yaw_list):
        for j, pitch_deg in enumerate(pitch_list):
            thrust_map[i, j], _ = find_max_thrust_for_orientation(
                alloc_mat, fg_w, 0.0, pitch_deg, yaw_deg, 0.0, 6 * THRUST_MAX, if_world)
        print(f"Completed yaw = {yaw_deg:.1f}°")

    if True:
        # save thrust map to file
        np.savez("thrust_map.npz", yaw_list=yaw_list, pitch_list=pitch_list, thrust_map=thrust_map)
        print("Saved thrust map to thrust_map.npz")

    # 4) flatten out directions & magnitudes
    dirs = []
    mags = []
    for i, yaw_deg in enumerate(yaw_list):
        for j, pitch_deg in enumerate(pitch_list):
            # compute world<-body rotation
            # TODO: there should be something wrong here, but I don't know why the result is correct
            R_bw = R.from_euler('zyx', [yaw_deg, pitch_deg, 0.0], degrees=True).as_matrix().T
            dir_z = R_bw[:, 2]  # world z-axis in body frame

            dirs.append(dir_z)
            mags.append(thrust_map[i, j])
    dirs = np.array(dirs)  # shape (N,3)
    mags = np.array(mags)  # shape (N,)

    # 5) prepare arrow components (all start at origin)
    origins = np.zeros_like(dirs)
    U = dirs[:, 0] * mags
    V = dirs[:, 1] * mags
    W = dirs[:, 2] * mags

    # 6) plot unit sphere and thrust vectors
    import scienceplots

    plt.style.use(["science"])

    plt.rcParams.update({'font.size': 11})  # default is 10
    label_size = 12

    fig = plt.figure(figsize=(3.5, 4))
    ax = fig.add_subplot(111, projection='3d')

    # calculate the 3D coordinates of the vector
    endpoints = origins + np.column_stack((U, V, W))
    xs, ys, zs = endpoints.T  # shape (N,)

    # 1) Reshape the endpoint coordinates into a 2-D grid using the (yaw, pitch) grid.
    Nyaw, Npitch = len(yaw_list), len(pitch_list)
    X = endpoints[:, 0].reshape(Nyaw, Npitch)
    Y = endpoints[:, 1].reshape(Nyaw, Npitch)
    Z = endpoints[:, 2].reshape(Nyaw, Npitch)

    # 2) Colormap (by thrust_map)
    norm = plt.Normalize(thrust_map.min(), thrust_map.max())
    colors = cm.viridis(norm(thrust_map))

    # 3) draw surface
    surf = ax.plot_surface(
        X, Y, Z,
        facecolors=colors,  # Each small grid has its own color
        rstride=1, cstride=1,  # Step size = 1 → draw every grid
        linewidth=0,
        antialiased=False,
        shade=False  # When using facecolors, turn off shade
    )

    # label axes and set equal aspect ratio
    coord_text = 'W' if if_world else 'B'
    ax.set_xlabel(f'$^{coord_text}f_x$ [N]', fontsize=label_size)
    ax.set_ylabel(f'$^{coord_text}f_y$ [N]', fontsize=label_size)
    ax.set_zlabel(f'$^{coord_text}f_z$ [N]', fontsize=label_size)
    # ax.set_title('Maximum Thrust Vectors on Unit Sphere Directions')
    ax.set_box_aspect((np.ptp(xs), np.ptp(ys), np.ptp(zs)))

    fig.subplots_adjust(left=0, right=0.95, bottom=0.15, top=1.0)

    # save the figure
    file_name = "max_force_world_3d.pdf" if if_world else "max_force_body_3d.pdf"
    fig.savefig(file_name, bbox_inches='tight', pad_inches=0.3)

    # 7) plot X-Z projection (new 2-D figure)
    fig2 = plt.figure(figsize=(3.5, 4))
    ax2 = fig2.add_subplot(111)

    sc2 = ax2.scatter(
        xs,  # X-coordinates
        zs,  # Z-coordinates
        c=mags,
        cmap='viridis',
        s=8
    )

    ax2.set_xlabel(f'$^{coord_text}f_x$ [N]', fontsize=label_size)
    ax2.set_ylabel(f'$^{coord_text}f_z$ [N]', fontsize=label_size)
    # ax2.set_title('X–Z Projection of Max-Thrust Directions')
    ax2.set_aspect('equal', adjustable='box')  # 保持比例尺一致
    ax2.grid(True)

    divider = make_axes_locatable(ax2)
    cax = divider.append_axes("bottom", size="6%", pad=0.5)
    cb = fig2.colorbar(sc2, cax=cax, orientation="horizontal")
    cb.set_label("Force (N)", fontsize=label_size)

    if if_world:
        plt.tight_layout(rect=[0, 0.0, 1.0, 1.1])
    else:
        plt.tight_layout(rect=[0, 0.15, 1.0, 1.0])

    plt.show()
