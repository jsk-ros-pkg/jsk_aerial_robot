#!/usr/bin/env python
# -*- encoding: ascii -*-
"""
Author: LI Jinjie
File: nmpc_body_rate_ctl.py
Date: 2023/4/16 9:50 AM
Description:
"""
import os
import sys
import shutil
import errno
import numpy as np
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver, AcadosModel
import casadi as ca

import nmpc_params as CP


class NMPCBodyRateController(object):
    def __init__(self, is_build_acados=True):
        opt_model = BodyRateModel().model

        nx = opt_model.x.size()[0]
        nu = opt_model.u.size()[0]
        n_params = opt_model.p.size()[0]

        # get file path for acados
        os.chdir(os.path.dirname(os.path.realpath(__file__)))
        acados_models_dir = "./acados_models"
        safe_mkdir_recursive(os.path.join(os.getcwd(), acados_models_dir))
        acados_source_path = os.environ["ACADOS_SOURCE_DIR"]
        sys.path.insert(0, acados_source_path)

        # create OCP
        ocp = AcadosOcp()
        ocp.acados_include_path = acados_source_path + "/include"
        ocp.acados_lib_path = acados_source_path + "/lib"
        ocp.model = opt_model
        ocp.dims.N = CP.N_node

        # initialize parameters
        ocp.dims.np = n_params
        ocp.parameter_values = np.zeros(n_params)

        # cost function
        # see https://docs.acados.org/python_interface/#acados_template.acados_ocp.AcadosOcpCost for details
        Q = np.diag([CP.Qp_xy, CP.Qp_xy, CP.Qp_z, CP.Qv_xy, CP.Qv_xy, CP.Qv_z, 0, CP.Qq_xy, CP.Qq_xy, CP.Qq_z])
        R = np.diag([CP.Rw, CP.Rw, CP.Rw, CP.Rc])
        ocp.cost.cost_type = "NONLINEAR_LS"
        ocp.cost.cost_type_e = "NONLINEAR_LS"
        ocp.cost.W = np.block([[Q, np.zeros((nx, nu))], [np.zeros((nu, nx)), R]])
        ocp.cost.W_e = Q  # weight matrix at terminal shooting node (N).

        # set constraints
        ocp.constraints.lbu = np.array([CP.w_min, CP.w_min, CP.w_min, CP.c_min])
        ocp.constraints.ubu = np.array([CP.w_max, CP.w_max, CP.w_max, CP.c_max])
        ocp.constraints.idxbu = np.array([0, 1, 2, 3])  # omega_x, omega_y, omega_z, collective_acceleration
        ocp.constraints.lbx = np.array([CP.v_min, CP.v_min, CP.v_min])
        ocp.constraints.ubx = np.array([CP.v_max, CP.v_max, CP.v_max])
        ocp.constraints.idxbx = np.array([3, 4, 5])  # vx, vy, vz

        # initial state
        x_ref = np.zeros(nx)
        u_ref = np.zeros(nu)
        ocp.constraints.x0 = x_ref
        ocp.cost.yref = np.concatenate((x_ref, u_ref))
        ocp.cost.yref_e = x_ref

        # solver options
        ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
        ocp.solver_options.hpipm_mode = "BALANCE"  # "BALANCE", "SPEED_ABS", "SPEED", "ROBUST". Default: "BALANCE".
        # # 0: no warm start; 1: warm start; 2: hot start. Default: 0   Seems only works for FULL_CONDENSING_QPOASES
        # ocp.solver_options.qp_solver_warm_start = 1
        ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
        ocp.solver_options.integrator_type = "ERK"  # explicit Runge-Kutta integrator
        ocp.solver_options.print_level = 0
        ocp.solver_options.nlp_solver_type = "SQP_RTI"
        ocp.solver_options.qp_solver_cond_N = CP.N_node
        ocp.solver_options.tf = CP.T_horizon

        # compile acados ocp
        json_file_path = os.path.join("./" + opt_model.name + "_acados_ocp.json")
        self.solver = AcadosOcpSolver(ocp, json_file=json_file_path, build=is_build_acados)

    def reset(self, xr, ur):
        # reset x and u of NNPC controller, which prevents warm-starting from the previous solution
        for i in range(self.solver.N):
            self.solver.set(i, "x", xr[i, :])
            self.solver.set(i, "u", ur[i, :])
        self.solver.set(self.solver.N, "x", xr[self.solver.N, :])

    def update(self, x0, xr, ur):
        # get x and u, set reference
        for i in range(self.solver.N):
            yr = np.concatenate((xr[i, :], ur[i, :]))
            self.solver.set(i, "yref", yr)
            quaternion_r = xr[i, 6:10]
            self.solver.set(i, "p", quaternion_r)  # for nonlinear quaternion error
        self.solver.set(self.solver.N, "yref", xr[self.solver.N, :])  # final state of x, no u

        # feedback, take the first action
        u0 = self.solver.solve_for_x0(x0)

        if self.solver.status != 0:
            raise Exception("acados acados_ocp_solver returned status {}. Exiting.".format(self.solver.status))

        return u0


class BodyRateModel(object):
    def __init__(self):
        model_name = "qd_body_rate_model"

        # model states
        x = ca.SX.sym("x")
        y = ca.SX.sym("y")
        z = ca.SX.sym("z")
        vx = ca.SX.sym("vx")
        vy = ca.SX.sym("vy")
        vz = ca.SX.sym("vz")
        qw = ca.SX.sym("qw")
        qx = ca.SX.sym("qx")
        qy = ca.SX.sym("qy")
        qz = ca.SX.sym("qz")
        states = ca.vertcat(x, y, z, vx, vy, vz, qw, qx, qy, qz)

        # reference for quaternions
        qwr = ca.SX.sym("qwr")
        qxr = ca.SX.sym("qxr")
        qyr = ca.SX.sym("qyr")
        qzr = ca.SX.sym("qzr")
        quaternion_r = ca.vertcat(qwr, qxr, qyr, qzr)

        # control inputs
        wx = ca.SX.sym("wx")
        wy = ca.SX.sym("wy")
        wz = ca.SX.sym("wz")
        c = ca.SX.sym("c")
        controls = ca.vertcat(wx, wy, wz, c)

        # dynamic model
        ds = ca.vertcat(
            vx,
            vy,
            vz,
            2 * (qx * qz + qw * qy) * c,
            2 * (qy * qz - qw * qx) * c,
            (1 - 2 * qx**2 - 2 * qy**2) * c - CP.gravity,
            (-wx * qx - wy * qy - wz * qz) * 0.5,
            (wx * qw + wz * qy - wy * qz) * 0.5,
            (wy * qw - wz * qx + wx * qz) * 0.5,
            (wz * qw + wy * qx - wx * qy) * 0.5,
        )

        # function
        f = ca.Function("f", [states, controls], [ds], ["state", "control_input"], ["ds"])

        # NONLINEAR_LS = error^T @ Q @ error; error = y - y_ref
        qe_x = qwr * qx - qw * qxr + qyr * qz - qy * qzr
        qe_y = qwr * qy - qw * qyr - qxr * qz + qx * qzr
        qe_z = qxr * qy - qx * qyr + qwr * qz - qw * qzr

        state_y = ca.vertcat(
            x,
            y,
            z,
            vx,
            vy,
            vz,
            qwr,
            qe_x + qxr,
            qe_y + qyr,
            qe_z + qzr,
        )
        control_y = controls

        # acados model
        x_dot = ca.SX.sym("x_dot", 10)
        f_impl = x_dot - f(states, controls)

        model = AcadosModel()
        model.name = model_name
        model.f_expl_expr = f(states, controls)  # CasADi expression for the explicit dynamics
        model.f_impl_expr = f_impl  # CasADi expression for the implicit dynamics
        model.x = states
        model.xdot = x_dot
        model.u = controls
        model.p = quaternion_r
        model.cost_y_expr = ca.vertcat(state_y, control_y)  # NONLINEAR_LS
        model.cost_y_expr_e = state_y

        # constraint
        constraint = ca.types.SimpleNamespace()

        constraint.w_max = CP.w_max
        constraint.w_min = CP.w_min
        constraint.c_max = CP.c_max
        constraint.c_min = CP.c_min

        constraint.v_max = CP.v_max
        constraint.v_min = CP.v_min
        constraint.expr = ca.vcat([wx, wy, wz, c])

        self.model = model
        self.constraint = constraint


def safe_mkdir_recursive(directory, overwrite=False):
    if not os.path.exists(directory):
        try:
            os.makedirs(directory)
        except OSError as exc:
            if exc.errno == errno.EEXIST and os.path.isdir(directory):
                pass
            else:
                raise
    else:
        if overwrite:
            try:
                shutil.rmtree(directory)
            except:
                print("Error while removing directory {}".format(directory))
