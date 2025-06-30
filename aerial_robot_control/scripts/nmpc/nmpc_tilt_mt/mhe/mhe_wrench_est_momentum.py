#!/usr/bin/env python
# -*- encoding: ascii -*-
import numpy as np
import casadi as ca
from acados_template import AcadosModel
from .mhe_base import MHEBase
from ..tilt_qd import phys_param_beetle_omni as phys_omni


class MHEWrenchEstMomentum(MHEBase):
    def __init__(self):
        # Read parameters from configuration file in the robot's package
        self.read_params("controller", "mhe", "beetle_omni", "WrenchEstMHEMomentum.yaml")
        self.phys = phys_omni

        super(MHEWrenchEstMomentum, self).__init__()

    def create_acados_model(self) -> AcadosModel:
        # Model name
        model_name = "mhe_wrench_est_momentum_mdl"

        # Model states
        v_w = ca.SX.sym("v_w", 3)               # Linear Velocity in World frame
        omega_b = ca.SX.sym("omega_b", 3)       # Angular Velocity in Body frame
        fds_w = ca.SX.sym("fds_w", 3)             # Disturbance on force in World frame
        tau_ds_b = ca.SX.sym("tau_ds_b", 3)       # Disturbance on torque in Body frame

        states = ca.vertcat(v_w, omega_b, fds_w, tau_ds_b)

        # Sensor function
        measurements = ca.vertcat(v_w, omega_b)

        # Model parameters
        f_u_w = ca.SX.sym("f_u_w", 3)
        tau_u_b = ca.SX.sym("tau_u_b", 3)

        controls = ca.vertcat(f_u_w, tau_u_b)   # Input u as parameter

        # Process noise on force and torque
        w_f = ca.SX.sym("w_f", 3)
        w_tau = ca.SX.sym("w_tau", 3)

        noise = ca.vertcat(w_f, w_tau)

        # Inertia
        I = ca.diag([self.phys.Ixx, self.phys.Iyy, self.phys.Izz])
        I_inv = ca.diag([1 / self.phys.Ixx, 1 / self.phys.Iyy, 1 / self.phys.Izz])
        g_w = np.array([0, 0, -self.phys.gravity])

        # Explicit dynamics
        ds = ca.vertcat(
            (f_u_w + fds_w) / self.phys.mass + g_w,
            ca.mtimes(I_inv, (-ca.cross(omega_b, ca.mtimes(I, omega_b)) + tau_u_b + tau_ds_b)),
            w_f,
            w_tau,
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
                self.params["R_v"],
                self.params["R_v"],
                self.params["R_v"],
                self.params["R_omega"],
                self.params["R_omega"],
                self.params["R_omega"],
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
                self.params["P_v"],
                self.params["P_v"],
                self.params["P_v"],
                self.params["P_omega"],
                self.params["P_omega"],
                self.params["P_omega"],
                self.params["P_f_d"],
                self.params["P_f_d"],
                self.params["P_f_d"],
                self.params["P_tau_d"],
                self.params["P_tau_d"],
                self.params["P_tau_d"],
            ]
        )
        print("Q_P: \n", Q_P)

        return Q_R, R_Q, Q_P


if __name__ == "__main__":
    print("Please run the gen_nmpc_code.py in the nmpc folder to generate the code for this estimator.")
