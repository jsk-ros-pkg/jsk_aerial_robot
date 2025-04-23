'''
 Created by li-jinjie on 25-4-23.
'''
import copy

import yaml
import os
import rospkg
import numpy as np

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


    print(alloc_mat)