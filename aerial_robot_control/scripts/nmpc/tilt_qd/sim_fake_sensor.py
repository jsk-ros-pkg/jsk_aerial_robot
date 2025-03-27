import numpy as np

from phys_param_beetle_omni import *


class FakeSensor:
    """
    Class to generate a mock-up of an acceleration sensor to simulate real feedback for EKF input.

    :param bool include_servo_model: Flag to include the servo model based on the angle alpha (a) between frame E (end of arm) and R (rotor). If not included, angle control is assumed to be equal to angle state.
    :param bool include_thrust_model: Flag to include dynamics from rotor and use thrust as state. If not included, thrust control is assumed to be equal to thrust state.
    :param bool include_cog_dist_model: Flag to include disturbance on the CoG into the acados model states. Disturbance on each rotor individually was investigated into but didn't properly work, therefore only disturbance on CoG implemented.
    """
    def __init__(self, include_servo_model: bool = False, 
                       include_thrust_model: bool = False,
                       include_cog_dist_model: bool = False):
        # Store controller flags
        self.include_servo_model = include_servo_model
        self.include_thrust_model = include_thrust_model
        self.include_cog_dist_model = include_cog_dist_model

        # Store physical properties
        self.mass = mass
        self.gravity = gravity

        self.I = np.diag([Ixx, Iyy, Izz])
        self.I_inv = np.diag([1 / Ixx, 1 / Iyy, 1 / Izz])
        self.g_i = np.array([0, 0, -gravity])

        self.dr1 = dr1
        self.dr2 = dr2
        self.dr3 = dr3
        self.dr4 = dr4
        self.p1_b = p1_b
        self.p2_b = p2_b
        self.p3_b = p3_b
        self.p4_b = p4_b
        self.kq_d_kt = kq_d_kt

        # Precompute rotation matrices
        denominator = np.sqrt(p1_b[0] ** 2 + p1_b[1] ** 2)
        self.rot_be1 = np.array([[p1_b[0] / denominator, -p1_b[1] / denominator, 0], [p1_b[1] / denominator, p1_b[0] / denominator, 0], [0, 0, 1]])
        denominator = np.sqrt(p2_b[0] ** 2 + p2_b[1] ** 2)
        self.rot_be2 = np.array([[p2_b[0] / denominator, -p2_b[1] / denominator, 0], [p2_b[1] / denominator, p2_b[0] / denominator, 0], [0, 0, 1]])
        denominator = np.sqrt(p3_b[0] ** 2 + p3_b[1] ** 2)
        self.rot_be3 = np.array([[p3_b[0] / denominator, -p3_b[1] / denominator, 0], [p3_b[1] / denominator, p3_b[0] / denominator, 0], [0, 0, 1]])
        denominator = np.sqrt(p4_b[0] ** 2 + p4_b[1] ** 2)
        self.rot_be4 = np.array([[p4_b[0] / denominator, -p4_b[1] / denominator, 0], [p4_b[1] / denominator, p4_b[0] / denominator, 0], [0, 0, 1]])

    def update_acc(self, x):
        # Deconstruct state variable
        # - Quaternions
        qw, qx, qy, qz = x[6:10]
        # - Angular velocity
        w = x[10:13]
        idx = 13
        # - Servo angle
        if self.include_servo_model:
            a1s, a2s, a3s, a4s = x[13:17]
            idx += 4
        # - Thrust
        if self.include_thrust_model:
            ft1s, ft2s, ft3s, ft4s = x[idx:idx+4]
            idx += 4
        # - Disturbance on CoG
        if self.include_cog_dist_model:
            fds_w = x[idx:idx+3]
            tau_ds_b = x[idx+3:idx+6]

        # Transformation matrix
        row_1 = np.array([1 - 2 * qy ** 2 - 2 * qz ** 2, 2 * qx * qy - 2 * qw * qz, 2 * qx * qz + 2 * qw * qy])
        row_2 = np.array([2 * qx * qy + 2 * qw * qz, 1 - 2 * qx ** 2 - 2 * qz ** 2, 2 * qy * qz - 2 * qw * qx])
        row_3 = np.array([2 * qx * qz - 2 * qw * qy, 2 * qy * qz + 2 * qw * qx, 1 - 2 * qx ** 2 - 2 * qy ** 2])
        rot_wb = np.vstack((row_1, row_2, row_3))
        rot_bw = rot_wb.T

        rot_e1r1 = self.rot_e2r(a1s)
        rot_e2r2 = self.rot_e2r(a2s)
        rot_e3r3 = self.rot_e2r(a3s)
        rot_e4r4 = self.rot_e2r(a4s)

        # Wrench in Rotor frame
        ft_r1 = np.array([0, 0, ft1s])
        ft_r2 = np.array([0, 0, ft2s])
        ft_r3 = np.array([0, 0, ft3s])
        ft_r4 = np.array([0, 0, ft4s])

        tau_r1 = np.array([0, 0, -self.dr1 * ft1s * self.kq_d_kt])
        tau_r2 = np.array([0, 0, -self.dr2 * ft2s * self.kq_d_kt])
        tau_r3 = np.array([0, 0, -self.dr3 * ft3s * self.kq_d_kt])
        tau_r4 = np.array([0, 0, -self.dr4 * ft4s * self.kq_d_kt])

        # Wrench in Body frame
        f_u_b = (
              np.dot(self.rot_be1, np.dot(rot_e1r1, ft_r1))
            + np.dot(self.rot_be2, np.dot(rot_e2r2, ft_r2))
            + np.dot(self.rot_be3, np.dot(rot_e3r3, ft_r3))
            + np.dot(self.rot_be4, np.dot(rot_e4r4, ft_r4))
        )

        tau_u_b = (
              np.dot(self.rot_be1, np.dot(rot_e1r1, tau_r1))
            + np.dot(self.rot_be2, np.dot(rot_e2r2, tau_r2))
            + np.dot(self.rot_be3, np.dot(rot_e3r3, tau_r3))
            + np.dot(self.rot_be4, np.dot(rot_e4r4, tau_r4))
            + np.cross(self.p1_b, np.dot(self.rot_be1, np.dot(rot_e1r1, ft_r1)))
            + np.cross(self.p2_b, np.dot(self.rot_be2, np.dot(rot_e2r2, ft_r2)))
            + np.cross(self.p3_b, np.dot(self.rot_be3, np.dot(rot_e3r3, ft_r3)))
            + np.cross(self.p4_b, np.dot(self.rot_be4, np.dot(rot_e4r4, ft_r4)))
        )

        # Specific Force in Body frame
        sf_b = (f_u_b + np.dot(rot_bw, fds_w)) / self.mass
        
        # Angular Acceleration in Body frame
        ang_acc_b = np.dot(self.I_inv, (-np.cross(w, np.dot(self.I, w)) + tau_u_b + tau_ds_b))

        return sf_b, ang_acc_b, rot_wb

    @staticmethod
    def rot_e2r(a):
        return np.array([[1, 0, 0], [0, np.cos(a), -np.sin(a)], [0, np.sin(a), np.cos(a)]])