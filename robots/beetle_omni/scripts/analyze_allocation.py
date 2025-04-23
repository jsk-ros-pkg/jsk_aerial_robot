'''
 Created by li-jinjie on 25-4-23.
'''
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

    # - Force
    alloc_mat[0, 0] = p1_b[1] / sqrt_p1b_xy
    alloc_mat[1, 0] = -p1_b[0] / sqrt_p1b_xy
    alloc_mat[2, 1] = 1

    alloc_mat[0, 2] = p2_b[1] / sqrt_p2b_xy
    alloc_mat[1, 2] = -p2_b[0] / sqrt_p2b_xy
    alloc_mat[2, 3] = 1

    alloc_mat[0, 4] = p3_b[1] / sqrt_p3b_xy
    alloc_mat[1, 4] = -p3_b[0] / sqrt_p3b_xy
    alloc_mat[2, 5] = 1

    alloc_mat[0, 6] = p4_b[1] / sqrt_p4b_xy
    alloc_mat[1, 6] = -p4_b[0] / sqrt_p4b_xy
    alloc_mat[2, 7] = 1

    # - Torque
    alloc_mat[3, 0] = -dr1 * kq_d_kt * p1_b[1] / sqrt_p1b_xy + p1_b[0] * p1_b[2] / sqrt_p1b_xy
    alloc_mat[4, 0] = dr1 * kq_d_kt * p1_b[0] / sqrt_p1b_xy + p1_b[1] * p1_b[2] / sqrt_p1b_xy
    alloc_mat[5, 0] = -p1_b[0] ** 2 / sqrt_p1b_xy - p1_b[1] ** 2 / sqrt_p1b_xy

    alloc_mat[3, 1] = p1_b[1]
    alloc_mat[4, 1] = -p1_b[0]
    alloc_mat[5, 1] = -dr1 * kq_d_kt

    alloc_mat[3, 2] = -dr2 * kq_d_kt * p2_b[1] / sqrt_p2b_xy + p2_b[0] * p2_b[2] / sqrt_p2b_xy
    alloc_mat[4, 2] = dr2 * kq_d_kt * p2_b[0] / sqrt_p2b_xy + p2_b[1] * p2_b[2] / sqrt_p2b_xy
    alloc_mat[5, 2] = -p2_b[0] ** 2 / sqrt_p2b_xy - p2_b[1] ** 2 / sqrt_p2b_xy

    alloc_mat[3, 3] = p2_b[1]
    alloc_mat[4, 3] = -p2_b[0]
    alloc_mat[5, 3] = -dr2 * kq_d_kt

    alloc_mat[3, 4] = -dr3 * kq_d_kt * p3_b[1] / sqrt_p3b_xy + p3_b[0] * p3_b[2] / sqrt_p3b_xy
    alloc_mat[4, 4] = dr3 * kq_d_kt * p3_b[0] / sqrt_p3b_xy + p3_b[1] * p3_b[2] / sqrt_p3b_xy
    alloc_mat[5, 4] = -p3_b[0] ** 2 / sqrt_p3b_xy - p3_b[1] ** 2 / sqrt_p3b_xy

    alloc_mat[3, 5] = p3_b[1]
    alloc_mat[4, 5] = -p3_b[0]
    alloc_mat[5, 5] = -dr3 * kq_d_kt

    alloc_mat[3, 6] = -dr4 * kq_d_kt * p4_b[1] / sqrt_p4b_xy + p4_b[0] * p4_b[2] / sqrt_p4b_xy
    alloc_mat[4, 6] = dr4 * kq_d_kt * p4_b[0] / sqrt_p4b_xy + p4_b[1] * p4_b[2] / sqrt_p4b_xy
    alloc_mat[5, 6] = -p4_b[0] ** 2 / sqrt_p4b_xy - p4_b[1] ** 2 / sqrt_p4b_xy

    alloc_mat[3, 7] = p4_b[1]
    alloc_mat[4, 7] = -p4_b[0]
    alloc_mat[5, 7] = -dr4 * kq_d_kt