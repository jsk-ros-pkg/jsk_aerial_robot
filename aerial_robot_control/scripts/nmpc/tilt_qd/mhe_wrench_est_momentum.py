#!/usr/bin/env python
# -*- encoding: ascii -*-
'''
 Created by jinjie on 24/11/29.
'''

import sys
import numpy as np
import yaml
import rospkg
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
import casadi as ca

from rh_base import RecedingHorizonBase

from phys_param_beetle_omni import *

# read parameters from yaml
rospack = rospkg.RosPack()

mhe_param_path = os.path.join(rospack.get_path("beetle_omni"), "config", "WrenchEstMHEMomentum.yaml")
with open(mhe_param_path, "r") as f:
    mhe_param_dict = yaml.load(f, Loader=yaml.FullLoader)
mhe_params = mhe_param_dict["controller"]["mhe"]
mhe_params["N_node"] = int(mhe_params["T_pred"] / mhe_params["T_integ"])


class MHEWrenchEstMomentum(RecedingHorizonBase):
    def __init__(self):
        super(MHEWrenchEstMomentum, self).__init__()
        self.t_servo = t_servo

    def set_name(self) -> str:
        model_name = "mhe_wrench_est_momentum_mdl"
        return model_name

    def create_acados_model(self, model_name: str) -> AcadosModel:
        # model states
        v_w = ca.SX.sym("v", 3)
        omega_g = ca.SX.sym("omega", 3)
        f_d_w = ca.SX.sym("fd", 3)  # world frame
        tau_d_g = ca.SX.sym("tau_d", 3)  # cog frame

        states = ca.vertcat(v_w, omega_g, f_d_w, tau_d_g)

        # sensor function
        measurements = ca.vertcat(v_w, omega_g)

        # parameters
        f_u_w = ca.SX.sym("f_u", 3)
        tau_u_g = ca.SX.sym("tau_u", 3)

        controls = ca.vertcat(f_u_w, tau_u_g)  # u

        # process noise
        w_f = ca.SX.sym("w_f", 3)
        w_tau = ca.SX.sym("w_tau", 3)

        noise = ca.vertcat(w_f, w_tau)

        # inertial
        iv = ca.diag([Ixx, Iyy, Izz])
        inv_iv = ca.diag([1 / Ixx, 1 / Iyy, 1 / Izz])
        g_i = np.array([0, 0, -gravity])

        # dynamic model
        ds = ca.vertcat(
            (f_u_w + f_d_w) / mass + g_i,
            ca.mtimes(inv_iv, (-ca.cross(omega_g, ca.mtimes(iv, omega_g)) + tau_u_g + tau_d_g)),
            w_f,
            w_tau,
        )

        # function
        func = ca.Function("func", [states, noise], [ds], ["state", "noise"], ["ds"], {"allow_free": True})

        # NONLINEAR_LS = error^T @ Q @ error; error = y - y_ref
        # acados model
        x_dot = ca.SX.sym("x_dot", states.size()[0])
        f_impl = x_dot - func(states, noise)

        model = AcadosModel()
        model.name = model_name
        model.f_expl_expr = func(states, noise)  # CasADi expression for the explicit dynamics
        model.f_impl_expr = f_impl  # CasADi expression for the implicit dynamics
        model.x = states
        model.xdot = x_dot
        model.u = noise
        model.p = controls

        model.cost_y_expr_0 = ca.vertcat(measurements, noise, states)  # y, u, x
        model.cost_y_expr = ca.vertcat(measurements, noise)  # y, u
        model.cost_y_expr_e = measurements  # y

        return model

    def create_acados_ocp_solver(self, ocp_model: AcadosModel, is_build: bool) -> AcadosOcpSolver:
        nx = ocp_model.x.size()[0]
        nw = ocp_model.u.size()[0]
        n_meas = ocp_model.cost_y_expr.size()[0] - nw
        n_params = ocp_model.p.size()[0]

        # get file path for acados
        rospack = rospkg.RosPack()
        folder_path = os.path.join(
            rospack.get_path("aerial_robot_control"), "include", "aerial_robot_control", "wrench_est", ocp_model.name
        )
        self._mkdir(folder_path)
        os.chdir(folder_path)
        # acados_models_dir = "acados_models"
        # safe_mkdir_recursive(os.path.join(os.getcwd(), acados_models_dir))

        acados_source_path = os.environ["ACADOS_SOURCE_DIR"]
        sys.path.insert(0, acados_source_path)

        # create OCP
        ocp = AcadosOcp()
        ocp.acados_include_path = acados_source_path + "/include"
        ocp.acados_lib_path = acados_source_path + "/lib"
        ocp.model = ocp_model
        ocp.dims.N = mhe_params["N_node"]

        # initialize parameters
        ocp.dims.np = n_params
        ocp.parameter_values = np.zeros(n_params)

        # cost function
        Q_P = np.diag(
            [
                mhe_params["P_v"],
                mhe_params["P_v"],
                mhe_params["P_v"],
                mhe_params["P_omega"],
                mhe_params["P_omega"],
                mhe_params["P_omega"],
                mhe_params["P_f_d"],
                mhe_params["P_f_d"],
                mhe_params["P_f_d"],
                mhe_params["P_tau_d"],
                mhe_params["P_tau_d"],
                mhe_params["P_tau_d"],
            ]
        )
        print("Q_P: \n", Q_P)

        Q_R = np.diag(
            [
                mhe_params["R_v"],
                mhe_params["R_v"],
                mhe_params["R_v"],
                mhe_params["R_omega"],
                mhe_params["R_omega"],
                mhe_params["R_omega"],
            ]
        )
        print("Q_R: \n", Q_R)

        R_Q = np.diag(
            [
                mhe_params["Q_w_f"],
                mhe_params["Q_w_f"],
                mhe_params["Q_w_f"],
                mhe_params["Q_w_tau"],
                mhe_params["Q_w_tau"],
                mhe_params["Q_w_tau"],
            ]
        )
        print("R_Q: \n", R_Q)

        ocp.cost.cost_type_0 = "NONLINEAR_LS"
        # concatenate the diagonal matrix: W_0 = diag(Q_R, R_Q, Q_P)
        W = np.block([[Q_R, np.zeros((n_meas, nw))], [np.zeros((nw, n_meas)), R_Q]])
        ocp.cost.W_0 = np.block([[W, np.zeros((n_meas + nw, nx))], [np.zeros((nx, n_meas + nw)), Q_P]])

        print("W_0: \n", ocp.cost.W_0)

        ocp.cost.cost_type = "NONLINEAR_LS"
        ocp.cost.W = np.block([[Q_R, np.zeros((n_meas, nw))], [np.zeros((nw, n_meas)), R_Q]])

        ocp.cost.cost_type_e = "NONLINEAR_LS"
        ocp.cost.W_e = Q_R  # weight matrix at terminal shooting node (N).

        # # set constraints

        # initial state
        ocp.cost.yref_0 = np.zeros(n_meas + nw + nx)
        ocp.cost.yref = np.zeros(n_meas + nw)
        ocp.cost.yref_e = np.zeros(n_meas)

        # solver options
        ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
        ocp.solver_options.hpipm_mode = "BALANCE"  # "BALANCE", "SPEED_ABS", "SPEED", "ROBUST". Default: "BALANCE".
        # # 0: no warm start; 1: warm start; 2: hot start. Default: 0   Seems only works for FULL_CONDENSING_QPOASES
        # ocp.solver_options.qp_solver_warm_start = 1
        ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
        ocp.solver_options.integrator_type = "ERK"  # explicit Runge-Kutta integrator
        ocp.solver_options.print_level = 0
        ocp.solver_options.nlp_solver_type = "SQP_RTI"
        ocp.solver_options.qp_solver_cond_N = mhe_params["N_node"]
        ocp.solver_options.tf = mhe_params["T_pred"]

        # compile acados ocp
        json_file_path = os.path.join("./" + ocp_model.name + "_acados_ocp.json")
        solver = AcadosOcpSolver(ocp, json_file=json_file_path, build=is_build)

        return solver


if __name__ == "__main__":
    nmpc = MHEWrenchEstMomentum()

    acados_ocp_solver = nmpc.get_ocp_solver()
    print("Successfully initialized acados ocp: ", acados_ocp_solver.acados_ocp)
    print("number of states: ", acados_ocp_solver.acados_ocp.dims.nx)
    print("number of controls: ", acados_ocp_solver.acados_ocp.dims.nu)
    print("number of parameters: ", acados_ocp_solver.acados_ocp.dims.np)
    print("T_samp: ", mhe_params["T_samp"])
    print("T_pred: ", mhe_params["T_pred"])
    print("T_integ: ", mhe_params["T_integ"])
    print("N_node: ", mhe_params["N_node"])
