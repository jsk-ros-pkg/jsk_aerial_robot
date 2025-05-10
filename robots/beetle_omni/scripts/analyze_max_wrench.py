'''
 Created by li-jinjie on 25-5-10.
'''
import numpy as np
import cvxpy as cp
from scipy.spatial.transform import Rotation as R

from analyze_allocation import get_alloc_mtx_tilt_qd, full_force_to_cmd
from analyze_allocation import mass, gravity

thrust_max = 30
thrust_min = 0


def check_wrench_available(alloc_mtx, tgt_wrench, f_th_max, wrench_error_limit=1e-3):
    alpha = 1e-3  # Tikhonov (small control effort penalty)

    # ---------- decision variable ----------
    x = cp.Variable(8)  # [Fx1,Fy1, Fx2,Fy2, Fx3,Fy3, Fx4,Fy4]

    # ---------- objective ----------
    objective = cp.Minimize(
        cp.sum_squares(alloc_mtx @ x - tgt_wrench[:, 0])  # track wrench
        # + alpha * cp.sum_squares(x)  # keep forces small
    )

    # ---------- constraints ----------
    constraints = []
    constraints += [x[0] ** 2 + x[1] ** 2 <= f_th_max ** 2]  # rotor 1
    constraints += [x[2] ** 2 + x[3] ** 2 <= f_th_max ** 2]  # rotor 2
    constraints += [x[4] ** 2 + x[5] ** 2 <= f_th_max ** 2]  # rotor 3
    constraints += [x[6] ** 2 + x[7] ** 2 <= f_th_max ** 2]  # rotor 4

    # ---------- solve ----------
    prob = cp.Problem(objective, constraints)
    prob.solve(verbose=False)  # choose solver='OSQP' if you like

    if prob.status != cp.OPTIMAL:
        raise RuntimeError("Problem is not optimal")

    # ---------- check if wrench is achieved ----------
    tgt_force = x.value.reshape(8, 1)
    ft_ref_local, a_ref_local = full_force_to_cmd(tgt_force)

    wrench_error_local = np.linalg.norm(alloc_mtx @ x.value - tgt_wrench[:, 0])

    is_available_local = True
    if wrench_error_local > wrench_error_limit:
        is_available_local = False

    return is_available_local, wrench_error_local, ft_ref_local, a_ref_local


if __name__ == "__main__":
    alloc_mat = get_alloc_mtx_tilt_qd()

    fg_w = np.array([0.0, 0.0, mass * gravity])  # gravity in world frame

    # PLEASE change here to traverse all the directions
    roll_deg = 0.0
    pitch_deg = 90.0
    yaw_deg = 10.0

    R_bw = R.from_euler('zyx', [yaw_deg, pitch_deg, roll_deg], degrees=True).as_matrix().T
    fg_b = R_bw @ fg_w

    # PLEASE change this part to find the max thrust along the direction
    desired_thrust = 200  # N

    tgt_w = np.array([[fg_b[0], fg_b[1], fg_b[2] + desired_thrust, 0.0, 0.0, 0.0]]).T

    is_available, wrench_error, ft_ref, a_ref = check_wrench_available(alloc_mat, tgt_w, thrust_max)

    if is_available:
        print("Wrench is available")
    else:
        print("Wrench is NOT available")

    print("wrench_error: ", wrench_error)
    print("ft_ref: ", ft_ref)
    print("a_ref: ", a_ref)
