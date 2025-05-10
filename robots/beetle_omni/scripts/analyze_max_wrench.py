'''
 Created by li-jinjie on 25-5-10.
'''
import numpy as np
import cvxpy as cp
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

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


def find_max_thrust_for_orientation(alloc_mtx, fg_w, roll_deg, pitch_deg,
                                    search_thrust_min, search_thrust_max,
                                    tol=1e-2, max_iters=20):
    """
    For a given roll and pitch angle (degrees), perform a binary search
    to find the maximum achievable additional thrust along the body z-axis.
    """
    yaw_deg = 0.0  # fix yaw, since gravity projection is yaw-invariant
    # compute rotation from world to body frame
    R_bw = R.from_euler('zyx', [yaw_deg, pitch_deg, roll_deg], degrees=True).as_matrix().T
    # gravity in body frame
    fg_b = R_bw @ fg_w

    lo, hi = search_thrust_min, search_thrust_max
    best = search_thrust_min
    for _ in range(max_iters):
        mid = (lo + hi) / 2
        # target wrench: include gravity plus additional thrust along body z
        tgt_w = np.array([[fg_b[0], fg_b[1], fg_b[2] + mid, 0.0, 0.0, 0.0]]).T
        ok, _ = check_wrench_available(alloc_mtx, tgt_w)
        if ok:
            best = mid
            lo = mid  # can try larger thrust
        else:
            hi = mid  # reduce thrust
        if hi - lo < tol:
            break
    return best


if __name__ == "__main__":
    # 1) prepare the allocation matrix and world gravity vector
    alloc_mat = get_alloc_mtx_tilt_qd()
    fg_w = np.array([0.0, 0.0, mass * gravity])

    # 2) define roll and pitch angle grids
    roll_list = np.linspace(-180, 180, 15)  # from -180° to 180°
    pitch_list = np.linspace(-90, 90, 5)  # from -90° to 90°

    # 3) compute max thrust map
    thrust_map = np.zeros((len(roll_list), len(pitch_list)))
    for i, roll_deg in enumerate(roll_list):
        for j, pitch_deg in enumerate(pitch_list):
            thrust_map[i, j] = find_max_thrust_for_orientation(
                alloc_mat, fg_w, roll_deg, pitch_deg, 4 * THRUST_MIN, 4 * THRUST_MAX,
            )
        print(f"Completed roll = {roll_deg:.1f}°")

    if True:
        # save thrust map to file
        np.savez("thrust_map.npz", roll_list=roll_list, pitch_list=pitch_list, thrust_map=thrust_map)
        print("Saved thrust map to thrust_map.npz")

    # 4) flatten out directions & magnitudes
    dirs = []
    mags = []
    for i, roll_deg in enumerate(roll_list):
        for j, pitch_deg in enumerate(pitch_list):
            # compute world<-body rotation
            R_wb = R.from_euler('zyx', [0.0, pitch_deg, roll_deg], degrees=True).as_matrix()
            dir_z = R_wb[:, 2]  # body z-axis in world frame
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
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # # draw a unit sphere wireframe for reference
    # u, v = np.mgrid[0:np.pi:40j, 0:2 * np.pi:40j]
    # x_s = np.sin(u) * np.cos(v)
    # y_s = np.sin(u) * np.sin(v)
    # z_s = np.cos(u)
    # ax.plot_wireframe(x_s, y_s, z_s, color='lightgray', alpha=0.5, linewidth=0.5)

    # plot thrust vectors from the origin
    ax.quiver(
        origins[:, 0], origins[:, 1], origins[:, 2],
        U, V, W,
        length=1.0,
        normalize=False,
        linewidth=0.5,
        pivot='tail'
    )

    # change lim
    max_thrust = mags.max()
    ax.set_xlim(-max_thrust, max_thrust)
    ax.set_ylim(-max_thrust, max_thrust)
    ax.set_zlim(-max_thrust, max_thrust)

    # label axes and set equal aspect ratio
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Maximum Thrust Vectors on Unit Sphere Directions')
    ax.set_box_aspect([1, 1, 1])

    plt.tight_layout()
    plt.show()
