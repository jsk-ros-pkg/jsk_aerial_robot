#!/usr/bin/env python
# -*- encoding: ascii -*-
import numpy as np
from acados_template import AcadosModel
import casadi as ca
from qd_mhe_base import QDMHEBase

from phys_param_beetle_omni import *


class MHEWrenchEstAccMom(QDMHEBase):
    def __init__(self):
        # Read parameters from configuration file in the robot's package
        self.read_params("controller", "mhe", "beetle_omni", "WrenchEstMHEAccMom.yaml")

        super(MHEWrenchEstAccMom, self).__init__()

    def create_acados_model(self) -> AcadosModel:
        # Model name
        model_name = "mhe_wrench_est_acc_mom_mdl"

        # Model states
        omega_b = ca.SX.sym("omega_b", 3)       # Angular Velocity in Body frame
        fds_w = ca.SX.sym("fds_w", 3)             # Disturbance on force in World frame
        tau_ds_b = ca.SX.sym("tau_ds_b", 3)       # Disturbance on torque in Body frame

        states = ca.vertcat(omega_b, fds_w, tau_ds_b)

        # Model parameters
        tau_u_b = ca.SX.sym("tau_u_b", 3)
        controls = ca.vertcat(tau_u_b)          # Input u as parameter

        # Process noise on force and torque
        w_f = ca.SX.sym("w_f", 3)
        w_tau = ca.SX.sym("w_tau", 3)

        noise = ca.vertcat(w_f, w_tau)

        # Sensor function
        measurements = ca.vertcat(fds_w, omega_b)

        # Inertia
        I = ca.diag([Ixx, Iyy, Izz])
        I_inv = ca.diag([1 / Ixx, 1 / Iyy, 1 / Izz])

        # Explicit dynamics
        ds = ca.vertcat(
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
                self.params["R_f_d"],
                self.params["R_f_d"],
                self.params["R_f_d"],
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
    mhe = MHEWrenchEstAccMom()

    acados_ocp_solver = mhe.get_ocp_solver()
    print("Successfully initialized acados ocp: ", acados_ocp_solver.acados_ocp)
    print("number of states: ", acados_ocp_solver.acados_ocp.dims.nx)
    print("number of controls: ", acados_ocp_solver.acados_ocp.dims.nu)
    print("number of parameters: ", acados_ocp_solver.acados_ocp.dims.np)
    print("T_samp: ", mhe.params["T_samp"])
    print("T_horizon: ", mhe.params["T_horizon"])
    print("T_step: ", mhe.params["T_step"])
    print("N_steps: ", mhe.params["N_steps"])
