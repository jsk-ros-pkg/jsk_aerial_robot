'''
 Created by li-jinjie on 25-4-23.
'''
import time
import yaml
import os
import rospkg
import numpy as np
from scipy.spatial.transform import Rotation as R
import cvxpy as cp
import matplotlib.pyplot as plt

# read parameters from yaml
rospack = rospkg.RosPack()

physical_param_path = os.path.join(rospack.get_path("beetle_omni"), "config", "PhysParamBeetleOmni.yaml")
with open(physical_param_path, "r") as f:
    physical_param_dict = yaml.load(f, Loader=yaml.FullLoader)
physical_params = physical_param_dict["physical"]

mass = physical_params["mass"]
gravity = physical_params["gravity"]
Ixx = physical_params["inertia_diag"][0]
Iyy = physical_params["inertia_diag"][1]
Izz = physical_params["inertia_diag"][2]
dr1 = physical_params["dr1"]
dr2 = physical_params["dr2"]
dr3 = physical_params["dr3"]
dr4 = physical_params["dr4"]
p1_b = physical_params["p1"]
p2_b = physical_params["p2"]
p3_b = physical_params["p3"]
p4_b = physical_params["p4"]
kq_d_kt = physical_params["kq_d_kt"]

t_servo = physical_params["t_servo"]  # time constant of servo
t_rotor = physical_params["t_rotor"]  # time constant of rotor

thrust_max = 30
thrust_min = 0
alpha_max = np.pi
alpha_min = -np.pi


def pseudoinverse_svd(mat, tolerance=1e-4):
    # Perform SVD
    U, singular_values, Vh = np.linalg.svd(mat, full_matrices=True)

    # Initialize inverse of the singular value matrix
    singular_values_inv = np.zeros((mat.shape[1], mat.shape[0]))

    # Fill in the inverse values conditionally
    for i in range(len(singular_values)):
        if singular_values[i] > tolerance:
            singular_values_inv[i, i] = 1.0 / singular_values[i]
        else:
            singular_values_inv[i, i] = 0.0

    # Compute pseudoinverse: V * Sigma⁻¹ * Uᴴ
    return Vh.T @ singular_values_inv @ U.T


def full_force_to_cmd(target_force):
    # Compute reference values for thrust
    # Set either state or control input based on model properties, i.e., based on include flags
    ft1_ref = np.sqrt(target_force[0, 0] ** 2 + target_force[1, 0] ** 2)
    ft2_ref = np.sqrt(target_force[2, 0] ** 2 + target_force[3, 0] ** 2)
    ft3_ref = np.sqrt(target_force[4, 0] ** 2 + target_force[5, 0] ** 2)
    ft4_ref = np.sqrt(target_force[6, 0] ** 2 + target_force[7, 0] ** 2)
    ft_ref = [ft1_ref, ft2_ref, ft3_ref, ft4_ref]

    # Compute reference values for servo angles
    # Set either state or control input based on model properties, i.e., based on include flags
    a1_ref = np.arctan2(target_force[0, 0], target_force[1, 0])
    a2_ref = np.arctan2(target_force[2, 0], target_force[3, 0])
    a3_ref = np.arctan2(target_force[4, 0], target_force[5, 0])
    a4_ref = np.arctan2(target_force[6, 0], target_force[7, 0])
    a_ref = [a1_ref, a2_ref, a3_ref, a4_ref]

    return ft_ref, a_ref


def get_cmd_w_inv_mat(inv_mat, tgt_wrench):
    target_force = inv_mat @ tgt_wrench
    return full_force_to_cmd(target_force)


def get_cmd_w_lstsq(alloc_mtx, tgt_wrench):
    target_force, _, _, _ = np.linalg.lstsq(alloc_mtx, tgt_wrench, rcond=None)
    return full_force_to_cmd(target_force)


def get_cmd_solve_qp(alloc_mtx, tgt_wrench, shutdown_rotor_idx=None, shutdown_rotor_fx=None, shutdown_rotor_fy=None):
    '''
    if only the objective is used as Ax-b (or add cp.sum_squares(x)), the result is the same as the lstsq method;
    :param alloc_mtx: allocation matrix
    :param tgt_wrench: target wrench
    :param shutdown_rotor_idx: rotor shutdown index, from 0 to 3
    '''
    alpha = 1e-3  # Tikhonov (small control effort penalty)

    # ---------- decision variable ----------
    x = cp.Variable(8)  # [Fx1,Fy1, Fx2,Fy2, Fx3,Fy3, Fx4,Fy4]

    # ---------- objective ----------
    objective = cp.Minimize(
        cp.sum_squares(alloc_mtx @ x - tgt_wrench[:, 0])  # track wrench
        + alpha * cp.sum_squares(x)  # keep forces small
    )

    # ---------- constraints ----------
    constraints = []

    if shutdown_rotor_idx is None:
        constraints += [x[3] >= 0]
    else:
        # rotor shutdown
        constraints += [x[2 * shutdown_rotor_idx] == shutdown_rotor_fx,
                        x[2 * shutdown_rotor_idx + 1] == shutdown_rotor_fy]

    # ---------- solve ----------
    prob = cp.Problem(objective, constraints)
    prob.solve(verbose=False)  # choose solver='OSQP' if you like

    if prob.status != cp.OPTIMAL:
        raise RuntimeError("allocation QP infeasible!")

    target_force = x.value.reshape(8, 1)
    return full_force_to_cmd(target_force)


if __name__ == "__main__":
    sqrt_p1b_xy = np.sqrt(p1_b[0] ** 2 + p1_b[1] ** 2)
    sqrt_p2b_xy = np.sqrt(p2_b[0] ** 2 + p2_b[1] ** 2)
    sqrt_p3b_xy = np.sqrt(p3_b[0] ** 2 + p3_b[1] ** 2)
    sqrt_p4b_xy = np.sqrt(p4_b[0] ** 2 + p4_b[1] ** 2)

    # Define Allocation Matrix
    alloc_mat = np.zeros((6, 8))

    # Rotor parameters in list form for looping
    p_b_list = [p1_b, p2_b, p3_b, p4_b]
    dr_list = [dr1, dr2, dr3, dr4]

    for i in range(4):
        p_b = p_b_list[i]
        sqrt_p_xy = np.sqrt(p_b[0] ** 2 + p_b[1] ** 2)
        dr = dr_list[i]

        # Force entries
        alloc_mat[0, 2 * i] = p_b[1] / sqrt_p_xy
        alloc_mat[1, 2 * i] = -p_b[0] / sqrt_p_xy
        alloc_mat[2, 2 * i + 1] = 1

        # Torque entries
        alloc_mat[3, 2 * i] = -dr * kq_d_kt * p_b[1] / sqrt_p_xy + p_b[0] * p_b[2] / sqrt_p_xy
        alloc_mat[4, 2 * i] = dr * kq_d_kt * p_b[0] / sqrt_p_xy + p_b[1] * p_b[2] / sqrt_p_xy
        alloc_mat[5, 2 * i] = -p_b[0] ** 2 / sqrt_p_xy - p_b[1] ** 2 / sqrt_p_xy

        alloc_mat[3, 2 * i + 1] = p_b[1]
        alloc_mat[4, 2 * i + 1] = -p_b[0]
        alloc_mat[5, 2 * i + 1] = -dr * kq_d_kt

    print("shape of alloc_mat", alloc_mat.shape)
    print("alloc_mat", alloc_mat)
    print("=======================\n")

    # ======= Test the pseudoinverse function ======
    # our method in the system to compute the allocation matrix
    alloc_mat_inv_svd = pseudoinverse_svd(alloc_mat)
    alloc_mat_inv_linalg = np.linalg.pinv(alloc_mat)
    alloc_mat_inv_right_inverse = alloc_mat.T @ np.linalg.inv(alloc_mat @ alloc_mat.T)

    print("===== alloc_mat_inv_linalg - alloc_mat_inv_svd =====")
    print(alloc_mat_inv_linalg - alloc_mat_inv_svd)
    print("===== alloc_mat_right_inverse - alloc_mat_inv_svd =====")
    print(alloc_mat_inv_right_inverse - alloc_mat_inv_svd)
    print("=======================\n")

    # ===== calculate thrust and servo angle =====
    fg_w = np.array([0, 0, mass * gravity])  # World frame
    rot = R.from_euler('zyx', [45, 90, 0], degrees=True).as_matrix()
    fg_b = rot.T @ fg_w  # Body frame
    print("fg_b", fg_b)
    print("The rotor 2 (index is 1 if counting from 0) should be pointing up")

    target_wrench = np.array([[fg_b.item(0), fg_b.item(1), fg_b.item(2), 0, 0, 0]]).T

    print("alloc_mat == alloc_mat_inv_svd")
    ft_ref, a_ref = get_cmd_w_inv_mat(alloc_mat_inv_svd, target_wrench)
    print("ft_ref", ft_ref)
    print("a_ref", a_ref)

    print("alloc_mat == alloc_mat_inv_linalg")
    ft_ref, a_ref = get_cmd_w_inv_mat(alloc_mat_inv_linalg, target_wrench)
    print("ft_ref", ft_ref)
    print("a_ref", a_ref)

    print("alloc_mat == alloc_mat_inv_right_inverse")
    ft_ref, a_ref = get_cmd_w_inv_mat(alloc_mat_inv_right_inverse, target_wrench)
    print("ft_ref", ft_ref)
    print("a_ref", a_ref)

    # -------------------------------------------------------------------
    # Weighted pseudo-inverse
    # pick a large weight for tilt components (entries 0,2,4,6)
    ratio = 1.1
    W = np.diag([ratio, 1.0, ratio, 1.0, ratio, 1.0, ratio, 1.0])
    A = alloc_mat
    Aw = A @ np.linalg.inv(W)
    inv_weighted_all = W @ A.T @ np.linalg.inv(Aw @ A.T)

    ratio = 1.5
    W = np.diag([1.0, 1.0, ratio, 1.0, 1.0, 1.0, 1.0, 1.0])
    A = alloc_mat
    Aw = A @ np.linalg.inv(W)
    inv_weighted_single = W @ A.T @ np.linalg.inv(Aw @ A.T)

    # PLOT THE RESULTS
    # -------------------------------------------------------------------
    # (1)  PSEUDOINVERSES FOR THE THREE ALLOCATION METHODS
    # -------------------------------------------------------------------
    inv_methods = {  # <--  this is what the patch uses
        "SVD": alloc_mat_inv_svd,
        # "np.pinv": alloc_mat_inv_linalg,
        "RightInverse": alloc_mat_inv_right_inverse,
        "LSTSQ": None,
        # "Weighted_all": inv_weighted_all,
        "Weighted_single": inv_weighted_single,
        "Constrained_QP": None,
        # "SVD+QP": None,
        "SVD+alloc": None
    }

    # -------------------------------------------------------------------
    # (2)  YAW-ANGLE SWEEP (deg)
    # -------------------------------------------------------------------
    yaw_deg = np.arange(40.0, 50.0 + 0.1, 0.1)  # 40° → 50° inclusive, 0.1° step

    # -------------------------------------------------------------------
    # (3)  ARRAYS THAT STORE RESULTS FOR EVERY METHOD AND EVERY YAW
    # -------------------------------------------------------------------
    ft_all = {key: np.zeros((len(yaw_deg), 4)) for key in inv_methods}  # thrust  (N)
    ang_all = {key: np.zeros((len(yaw_deg), 4)) for key in inv_methods}  # servo angles (rad)

    fg_w = np.array([0.0, 0.0, mass * gravity])  # gravity in world frame

    for key, inv in inv_methods.items():
        time_start = time.time()

        rotor_idx_prev = -1
        alloc_mat_del_rotor_inv = None
        for idx, yaw in enumerate(yaw_deg):
            # body-frame gravity for roll=0, pitch=90°, varying yaw
            R_bw = R.from_euler('zyx', [yaw, 90.0, 0.0], degrees=True).as_matrix().T
            fg_b = R_bw @ fg_w
            tgt_w = np.array([[fg_b[0], fg_b[1], fg_b[2], 0.0, 0.0, 0.0]]).T

            if inv is None:
                if "LSTSQ" in key:
                    ft_ref, a_ref = get_cmd_w_lstsq(alloc_mat, tgt_w)
                elif "Constrained_QP" in key:
                    ft_ref, a_ref = get_cmd_solve_qp(alloc_mat, tgt_w)
            else:
                ft_ref, a_ref = get_cmd_w_inv_mat(inv, tgt_w)

            if "SVD+" in key:
            # 1) do one allocation w. SVD
                target_force = alloc_mat_inv_svd @ tgt_w
                ft_ref, a_ref = full_force_to_cmd(target_force)
                # 2) check if one rotor's thrust is less than threshold and flip backwards
                ft_thresh = 1.0  # N
                rotor_idx_ft_cond = np.where(np.array(ft_ref) < ft_thresh)
                rotor_idx_alpha_cond = np.where((np.array(a_ref) > np.pi / 2) | (np.array(a_ref) < -np.pi / 2))
                rotor_idx = np.intersect1d(rotor_idx_ft_cond[0], rotor_idx_alpha_cond[0])

                if len(rotor_idx) > 1:
                    raise RuntimeError("More than one rotor is below threshold and flip backwards!")
                elif len(rotor_idx) == 0:
                    rotor_idx = -1
                elif len(rotor_idx) == 1:
                    rotor_idx = rotor_idx[0]

                # 3) if rotor_idx is not empty, maintain the thrust and modify the angle
                if rotor_idx != -1:
                    ft_stop_rotor = ft_ref[rotor_idx]
                    alpha_stop_rotor = np.pi / 2.0 - np.acos(target_force[2 * rotor_idx, 0] / ft_thresh)
                    ft_stop_rotor_x = ft_stop_rotor * np.sin(alpha_stop_rotor)
                    ft_stop_rotor_y = ft_stop_rotor * np.cos(alpha_stop_rotor)

                    if "+QP" in key:
                        # 4) do a QP with this rotor shutdown
                        ft_ref, a_ref = get_cmd_solve_qp(alloc_mat, tgt_w, rotor_idx,
                                                         ft_stop_rotor_x, ft_stop_rotor_y)
                    elif "+alloc" in key:
                        # 4.1) construct tgt_wrench from z_from_rotor
                        z_from_rotor = np.zeros((8, 1))
                        z_from_rotor[2 * rotor_idx] = ft_stop_rotor_x
                        z_from_rotor[2 * rotor_idx + 1] = ft_stop_rotor_y
                        tgt_wrench_from_rotor = alloc_mat @ z_from_rotor

                        # 4.2) calculate alloc_mat with this rotor's contribution
                        tgt_wrench_modified = tgt_w - tgt_wrench_from_rotor

                        # 4.3) calculate the allocation matrix without this rotor, which is 6*6
                        if rotor_idx != rotor_idx_prev:
                            alloc_mat_del_rotor = np.delete(alloc_mat, [2 * rotor_idx, 2 * rotor_idx + 1], axis=1)
                            alloc_mat_del_rotor_inv = np.linalg.inv(alloc_mat_del_rotor)

                        # 4.4) reconstruct the z output
                        z_except_rotor = alloc_mat_del_rotor_inv @ tgt_wrench_modified

                        # 4.5) at the place of 2*rotor_idx, insert 2 numbers to z_except_rotor
                        z_final = np.insert(z_except_rotor, 2 * rotor_idx,
                                            [[ft_stop_rotor_x], [ft_stop_rotor_y]], axis=0)

                        # 4.6) reconstruct the thrust and servo angle
                        ft_ref, a_ref = full_force_to_cmd(z_final)

                rotor_idx_prev = rotor_idx


            ft_all[key][idx, :] = ft_ref
            ang_all[key][idx, :] = a_ref

        time_end = time.time()
        print(f"Method: {key}, Average time: {(time_end - time_start) / len(yaw_deg) * 1000} ms")

    # ---------------------------------------------------------------
    # 6)  PLOTTING  (4×2 grid)
    # ---------------------------------------------------------------
    method_colors = {"SVD": "tab:orange",
                     "np.pinv": "tab:blue",
                     "RightInverse": "tab:red",
                     "LSTSQ": "tab:gray",
                     "Weighted_all": "tab:green",
                     "Weighted_single": "tab:purple",
                     "Constrained_QP": "tab:cyan",
                     "SVD+QP": "tab:brown",
                     "SVD+alloc": "tab:green"}
    rotor_names = [f"Rotor {i + 1}" for i in range(4)]

    fig, axs = plt.subplots(4, 2, figsize=(12, 14), sharex=True)
    fig.suptitle("Thrust & Servo-angle vs yaw (40°→50°)\nThree allocation inverses", fontsize=16)

    for r in range(4):
        # ---- thrust subplot (left) ----
        ax_t = axs[r, 0]
        for m in inv_methods:
            ax_t.plot(yaw_deg, ft_all[m][:, r], label=m if r == 0 else "",
                      color=method_colors[m])
        ax_t.set_ylabel("Thrust [N]")
        ax_t.set_title(f"{rotor_names[r]} thrust")
        ax_t.grid(True, linestyle=":")

        # ---- servo-angle subplot (right) ----
        ax_a = axs[r, 1]
        for m in inv_methods:
            ax_a.plot(yaw_deg, ang_all[m][:, r], label=m if r == 0 else "",
                      color=method_colors[m])
        ax_a.set_ylabel("Servo angle [rad]")
        ax_a.set_title(f"{rotor_names[r]} servo angle")
        ax_a.grid(True, linestyle=":")

    # shared x-label (bottom row only)
    for ax in axs[-1, :]:
        ax.set_xlabel("Yaw [deg]")

    # one legend outside the grid
    handles, labels = axs[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper right", bbox_to_anchor=(0.97, 0.97))

    plt.tight_layout(rect=[0, 0, 0.95, 0.96])
    plt.show()
