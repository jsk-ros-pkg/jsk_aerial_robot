#!/usr/bin/env python
# -*- encoding: ascii -*-
import numpy as np
from acados_template import AcadosModel
import casadi as ca
from qd_mhe_base import QDMHEBase

from phys_param_beetle_omni import *


class MHEWrenchEstIMUAct(QDMHEBase):
    def __init__(self):
        # Read parameters from configuration file in the robot's package
        self.read_params("controller", "mhe", "beetle_omni", "WrenchEstMHEImuActuator.yaml")

        super(MHEWrenchEstIMUAct, self).__init__()

    def create_acados_model(self) -> AcadosModel:
        # Model name
        model_name = "mhe_wrench_est_imu_act_mdl"

        # Model states
        wx = ca.SX.sym("wx")      # Angular velocity
        wy = ca.SX.sym("wy")
        wz = ca.SX.sym("wz")
        w = ca.vertcat(wx, wy, wz)

        a1s = ca.SX.sym("a1s")    # Servo angle
        a2s = ca.SX.sym("a2s")
        a3s = ca.SX.sym("a3s")
        a4s = ca.SX.sym("a4s")
        a_s = ca.vertcat(a1s, a2s, a3s, a4s)

        ft1s = ca.SX.sym("ft1s")  # Thrust force
        ft2s = ca.SX.sym("ft2s")
        ft3s = ca.SX.sym("ft3s")
        ft4s = ca.SX.sym("ft4s")
        ft_s = ca.vertcat(ft1s, ft2s, ft3s, ft4s)

        fds_w = ca.SX.sym("fds_w", 3)         # Disturbance on force in World frame
        tau_ds_b = ca.SX.sym("tau_ds_b", 3)   # Disturbance on torque in Body frame

        states = ca.vertcat(w, fds_w, tau_ds_b, a_s, ft_s)

        # Process noise on force and torque
        w_f = ca.SX.sym("w_f", 3)
        w_tau = ca.SX.sym("w_tau", 3)

        noise = ca.vertcat(w_f, w_tau)

        # Model parameters
        qw = ca.SX.sym("qw")
        qx = ca.SX.sym("qx")
        qy = ca.SX.sym("qy")
        qz = ca.SX.sym("qz")
        q = ca.vertcat(qw, qx, qy, qz)

        ft1c = ca.SX.sym("ft1c")
        ft2c = ca.SX.sym("ft2c")
        ft3c = ca.SX.sym("ft3c")
        ft4c = ca.SX.sym("ft4c")
        ft_c = ca.vertcat(ft1c, ft2c, ft3c, ft4c)

        a1c = ca.SX.sym("a1c")
        a2c = ca.SX.sym("a2c")
        a3c = ca.SX.sym("a3c")
        a4c = ca.SX.sym("a4c")
        a_c = ca.vertcat(a1c, a2c, a3c, a4c)

        controls = ca.vertcat(q, ft_c, a_c)   # Input u as parameters

        # Transformation matrix
        row_1 = ca.horzcat(
            ca.SX(1 - 2 * qy ** 2 - 2 * qz ** 2), ca.SX(2 * qx * qy - 2 * qw * qz), ca.SX(2 * qx * qz + 2 * qw * qy)
        )
        row_2 = ca.horzcat(
            ca.SX(2 * qx * qy + 2 * qw * qz), ca.SX(1 - 2 * qx ** 2 - 2 * qz ** 2), ca.SX(2 * qy * qz - 2 * qw * qx)
        )
        row_3 = ca.horzcat(
            ca.SX(2 * qx * qz - 2 * qw * qy), ca.SX(2 * qy * qz + 2 * qw * qx), ca.SX(1 - 2 * qx ** 2 - 2 * qy ** 2)
        )
        rot_wb = ca.vertcat(row_1, row_2, row_3)

        rot_bw = rot_wb.T

        den = np.sqrt(p1_b[0] ** 2 + p1_b[1] ** 2)
        rot_be1 = np.array([[p1_b[0] / den, -p1_b[1] / den, 0], [p1_b[1] / den, p1_b[0] / den, 0], [0, 0, 1]])

        den = np.sqrt(p2_b[0] ** 2 + p2_b[1] ** 2)
        rot_be2 = np.array([[p2_b[0] / den, -p2_b[1] / den, 0], [p2_b[1] / den, p2_b[0] / den, 0], [0, 0, 1]])

        den = np.sqrt(p3_b[0] ** 2 + p3_b[1] ** 2)
        rot_be3 = np.array([[p3_b[0] / den, -p3_b[1] / den, 0], [p3_b[1] / den, p3_b[0] / den, 0], [0, 0, 1]])

        den = np.sqrt(p4_b[0] ** 2 + p4_b[1] ** 2)
        rot_be4 = np.array([[p4_b[0] / den, -p4_b[1] / den, 0], [p4_b[1] / den, p4_b[0] / den, 0], [0, 0, 1]])

        rot_e1r1 = ca.vertcat(
            ca.horzcat(1, 0, 0), ca.horzcat(0, ca.cos(a1s), -ca.sin(a1s)), ca.horzcat(0, ca.sin(a1s), ca.cos(a1s))
        )
        rot_e2r2 = ca.vertcat(
            ca.horzcat(1, 0, 0), ca.horzcat(0, ca.cos(a2s), -ca.sin(a2s)), ca.horzcat(0, ca.sin(a2s), ca.cos(a2s))
        )
        rot_e3r3 = ca.vertcat(
            ca.horzcat(1, 0, 0), ca.horzcat(0, ca.cos(a3s), -ca.sin(a3s)), ca.horzcat(0, ca.sin(a3s), ca.cos(a3s))
        )
        rot_e4r4 = ca.vertcat(
            ca.horzcat(1, 0, 0), ca.horzcat(0, ca.cos(a4s), -ca.sin(a4s)), ca.horzcat(0, ca.sin(a4s), ca.cos(a4s))
        )

        # Wrench
        ft_r1 = ca.vertcat(0, 0, ft1s)
        ft_r2 = ca.vertcat(0, 0, ft2s)
        ft_r3 = ca.vertcat(0, 0, ft3s)
        ft_r4 = ca.vertcat(0, 0, ft4s)

        tau_r1 = ca.vertcat(0, 0, -dr1 * ft1s * kq_d_kt)
        tau_r2 = ca.vertcat(0, 0, -dr2 * ft2s * kq_d_kt)
        tau_r3 = ca.vertcat(0, 0, -dr3 * ft3s * kq_d_kt)
        tau_r4 = ca.vertcat(0, 0, -dr4 * ft4s * kq_d_kt)

        f_u_b = (
                  ca.mtimes(rot_be1, ca.mtimes(rot_e1r1, ft_r1))
                + ca.mtimes(rot_be2, ca.mtimes(rot_e2r2, ft_r2))
                + ca.mtimes(rot_be3, ca.mtimes(rot_e3r3, ft_r3))
                + ca.mtimes(rot_be4, ca.mtimes(rot_e4r4, ft_r4))
        )
        tau_u_b = (
                  ca.mtimes(rot_be1, ca.mtimes(rot_e1r1, tau_r1))
                + ca.mtimes(rot_be2, ca.mtimes(rot_e2r2, tau_r2))
                + ca.mtimes(rot_be3, ca.mtimes(rot_e3r3, tau_r3))
                + ca.mtimes(rot_be4, ca.mtimes(rot_e4r4, tau_r4))
                + ca.cross(np.array(p1_b), ca.mtimes(rot_be1, ca.mtimes(rot_e1r1, ft_r1)))
                + ca.cross(np.array(p2_b), ca.mtimes(rot_be2, ca.mtimes(rot_e2r2, ft_r2)))
                + ca.cross(np.array(p3_b), ca.mtimes(rot_be3, ca.mtimes(rot_e3r3, ft_r3)))
                + ca.cross(np.array(p4_b), ca.mtimes(rot_be4, ca.mtimes(rot_e4r4, ft_r4)))
        )

        # Sensor function
        measurements = ca.vertcat(
            (f_u_b + ca.mtimes(rot_bw, fds_w)) / mass,
            w,
            a_s,
            ft_s
        )

        # Inertia
        I = ca.diag([Ixx, Iyy, Izz])
        I_inv = ca.diag([1 / Ixx, 1 / Iyy, 1 / Izz])

        # Explicit dynamics
        ds = ca.vertcat(
            ca.mtimes(I_inv, (-ca.cross(w, ca.mtimes(I, w)) + tau_u_b + tau_ds_b)),
            w_f,
            w_tau,
            (a_c - a_s) / t_servo,
            (ft_c - ft_s) / t_rotor
        )
        f = ca.Function("f", [states, noise], [ds], ["state", "noise"], ["ds"], {"allow_free": True})

        # Implicit dynamics
        x_dot = ca.SX.sym("x_dot", states.size()[0])
        f_impl = x_dot - f(states, noise)

        # Assemble acados model
        model = AcadosModel()
        model.name = model_name
        model.f_expl_expr = f(states, noise)    # CasADi expression for the explicit dynamics
        model.f_impl_expr = f_impl              # CasADi expression for the implicit dynamics
        model.x = states
        model.xdot = x_dot
        model.u = noise
        model.p = controls
        
        # Cost function
        # error = y - y_ref
        # NONLINEAR_LS = error^T @ Q @ error
        model.cost_y_expr_0 = ca.vertcat(measurements, noise, states)  # y, u, x
        model.cost_y_expr = ca.vertcat(measurements, noise)  # y, u
        model.cost_y_expr_e = measurements  # y

        return model

    def get_weights(self):
        # Weights
        Q_R = np.diag(
            [
                1 / (self.params["R_a"] ** 2),
                1 / (self.params["R_a"] ** 2),
                1 / (self.params["R_a"] ** 2),
                1 / (self.params["R_omega"] ** 2),
                1 / (self.params["R_omega"] ** 2),
                1 / (self.params["R_omega"] ** 2),
                1 / (self.params["R_alpha"] ** 2),
                1 / (self.params["R_alpha"] ** 2),
                1 / (self.params["R_alpha"] ** 2),
                1 / (self.params["R_alpha"] ** 2),
                1 / (self.params["R_ft"] ** 2),
                1 / (self.params["R_ft"] ** 2),
                1 / (self.params["R_ft"] ** 2),
                1 / (self.params["R_ft"] ** 2),
            ]
        )
        print("Q_R: \n", Q_R)

        R_Q = np.diag(
            [
                self.params["Q_w_f"],
                self.params["Q_w_f"],
                self.params["Q_w_f"],
                self.params["Q_w_tau"],
                self.params["Q_w_tau"],
                self.params["Q_w_tau"],
            ]
        )
        print("R_Q: \n", R_Q)

        Q_P = np.diag(
            [
                self.params["P_omega"],
                self.params["P_omega"],
                self.params["P_omega"],
                self.params["P_f_d"],
                self.params["P_f_d"],
                self.params["P_f_d"],
                self.params["P_tau_d"],
                self.params["P_tau_d"],
                self.params["P_tau_d"],
                self.params["P_alpha"],
                self.params["P_alpha"],
                self.params["P_alpha"],
                self.params["P_alpha"],
                self.params["P_ft"],
                self.params["P_ft"],
                self.params["P_ft"],
                self.params["P_ft"],
            ]
        )
        print("Q_P: \n", Q_P)

        return Q_R, R_Q, Q_P


if __name__ == "__main__":
    mhe = MHEWrenchEstIMUAct()

    acados_ocp_solver = mhe.get_ocp_solver()
    print("Successfully initialized acados ocp: ", acados_ocp_solver.acados_ocp)
    print("number of states: ", acados_ocp_solver.acados_ocp.dims.nx)
    print("number of controls: ", acados_ocp_solver.acados_ocp.dims.nu)
    print("number of parameters: ", acados_ocp_solver.acados_ocp.dims.np)
    print("T_samp: ", mhe.params["T_samp"])
    print("T_horizon: ", mhe.params["T_horizon"])
    print("T_step: ", mhe.params["T_step"])
    print("N_steps: ", mhe.params["N_steps"])
