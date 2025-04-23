'''
 Created by li-jinjie on 25-4-23.
'''
import copy

import yaml
import os
import rospkg
import numpy as np
from scipy.spatial.transform import Rotation as R

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

def calculate_cmd(inv_mat, tgt_wrench):
    # A faster method if alloc_mat is dynamic:  x, _, _, _ = np.linalg.lstsq(alloc_mat, target_wrench, rcond=None)
    target_force = inv_mat @ tgt_wrench

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

    print("ft_ref", ft_ref)
    print("a_ref", a_ref)



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
    print("=======================")

    # ======= Test the pseudoinverse function ======
    # our method in the system to compute the allocation matrix
    alloc_mat_inv_svd = pseudoinverse_svd(alloc_mat)
    alloc_mat_inv_linalg = np.linalg.pinv(alloc_mat)
    alloc_mat_inv_right_inverse = alloc_mat.T @ np.linalg.inv(alloc_mat @ alloc_mat.T)

    print("===== alloc_mat_inv_linalg - alloc_mat_inv_svd =====")
    print(alloc_mat_inv_linalg - alloc_mat_inv_svd)
    print("===== alloc_mat_right_inverse - alloc_mat_inv_svd =====")
    print(alloc_mat_inv_right_inverse - alloc_mat_inv_svd)

    # ===== calculate thrust and servo angle =====
    fg_w = np.array([0, 0, mass * gravity])  # World frame
    rot = R.from_euler('zyx', [45, 90, 0], degrees=True).as_matrix()
    fg_b = rot.T @ fg_w  # Body frame
    print("fg_b", fg_b)
    print("The rotor 2 (index is 1 if counting from 0) should be pointing up")

    target_wrench = np.array([[fg_b.item(0), fg_b.item(1), fg_b.item(2), 0, 0, 0]]).T

    print("alloc_mat == alloc_mat_inv_svd")
    calculate_cmd(alloc_mat_inv_svd, target_wrench)

    print("alloc_mat == alloc_mat_inv_linalg")
    calculate_cmd(alloc_mat_inv_linalg, target_wrench)

    print("alloc_mat == alloc_mat_inv_right_inverse")
    calculate_cmd(alloc_mat_inv_right_inverse, target_wrench)

