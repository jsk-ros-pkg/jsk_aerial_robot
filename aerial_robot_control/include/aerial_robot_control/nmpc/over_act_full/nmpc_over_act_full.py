#!/usr/bin/env python
# -*- encoding: ascii -*-
"""
Author: LI Jinjie
File: nmpc_over_act_full.py
Date: 2023/11/27 9:43 PM
Description: the output of the NMPC controller is the thrust for each rotor and the servo angle for each servo
"""
from __future__ import print_function  # be compatible with python2
import os
import sys
import shutil
import errno
import numpy as np
import yaml
import rospkg
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver, AcadosModel
import casadi as ca

# read parameters from yaml
rospack = rospkg.RosPack()
param_path = os.path.join(rospack.get_path("beetle"), "config", "BeetleNMPCControl_sim.yaml")
with open(param_path, "r") as f:
    param_dict = yaml.load(f, Loader=yaml.FullLoader)

nmpc_params = param_dict["controller"]["nmpc"]
nmpc_params["N_node"] = int(nmpc_params["T_pred"] / nmpc_params["T_integ"])

physical_params = param_dict["controller"]["physical"]
mass = physical_params["mass"]
gravity = physical_params["gravity"]
Ixx = physical_params["inertia_diag"][0]
Iyy = physical_params["inertia_diag"][1]
Izz = physical_params["inertia_diag"][2]
dr1 = physical_params["dr1"]
p1 = physical_params["p1"]
dr2 = physical_params["dr2"]
p2 = physical_params["p2"]
dr3 = physical_params["dr3"]
p3 = physical_params["p3"]
dr4 = physical_params["dr4"]
p4 = physical_params["p4"]
kq_d_kt = physical_params["kq_d_kt"]

t_servo = 0.1  # time constant of servo


class NMPCController(object):
    def __init__(self):
        opt_model = BeetleFullModel().model

        nx = opt_model.x.size()[0]
        nu = opt_model.u.size()[0]
        n_params = opt_model.p.size()[0]

        # get file path for acados
        os.chdir(os.path.dirname(os.path.realpath(__file__)))
        acados_models_dir = "acados_models"
        safe_mkdir_recursive(os.path.join(os.getcwd(), acados_models_dir))
        acados_source_path = os.environ["ACADOS_SOURCE_DIR"]
        sys.path.insert(0, acados_source_path)

        # create OCP
        ocp = AcadosOcp()
        ocp.acados_include_path = acados_source_path + "/include"
        ocp.acados_lib_path = acados_source_path + "/lib"
        ocp.model = opt_model
        ocp.dims.N = nmpc_params["N_node"]

        # initialize parameters
        ocp.dims.np = n_params
        ocp.parameter_values = np.zeros(n_params)

        # cost function
        # see https://docs.acados.org/python_interface/#acados_template.acados_ocp.AcadosOcpCost for details
        Q = np.diag(
            [
                nmpc_params["Qp_xy"],
                nmpc_params["Qp_xy"],
                nmpc_params["Qp_z"],
                nmpc_params["Qv_xy"],
                nmpc_params["Qv_xy"],
                nmpc_params["Qv_z"],
                0,
                nmpc_params["Qq_xy"],
                nmpc_params["Qq_xy"],
                nmpc_params["Qq_z"],
                nmpc_params["Qw_xy"],
                nmpc_params["Qw_xy"],
                nmpc_params["Qw_z"],
            ]
        )
        R = np.diag([nmpc_params["Rt"], nmpc_params["Rt"], nmpc_params["Rt"], nmpc_params["Rt"]])
        ocp.cost.cost_type = "NONLINEAR_LS"
        ocp.cost.cost_type_e = "NONLINEAR_LS"
        ocp.cost.W = np.block([[Q, np.zeros((nx, nu))], [np.zeros((nu, nx)), R]])
        ocp.cost.W_e = Q  # weight matrix at terminal shooting node (N).

        # set constraints
        # # bx
        ocp.constraints.idxbx = np.array([3, 4, 5, 10, 11, 12])  # vx, vy, vz, wx, wy, wz
        ocp.constraints.lbx = np.array(
            [
                nmpc_params["v_min"],
                nmpc_params["v_min"],
                nmpc_params["v_min"],
                nmpc_params["w_min"],
                nmpc_params["w_min"],
                nmpc_params["w_min"],
            ]
        )
        ocp.constraints.ubx = np.array(
            [
                nmpc_params["v_max"],
                nmpc_params["v_max"],
                nmpc_params["v_max"],
                nmpc_params["w_max"],
                nmpc_params["w_max"],
                nmpc_params["w_max"],
            ]
        )

        # # bx_e
        ocp.constraints.idxbx_e = np.array([3, 4, 5, 10, 11, 12])  # vx, vy, vz, wx, wy, wz
        ocp.constraints.lbx_e = np.array(
            [
                nmpc_params["v_min"],
                nmpc_params["v_min"],
                nmpc_params["v_min"],
                nmpc_params["w_min"],
                nmpc_params["w_min"],
                nmpc_params["w_min"],
            ]
        )
        ocp.constraints.ubx_e = np.array(
            [
                nmpc_params["v_max"],
                nmpc_params["v_max"],
                nmpc_params["v_max"],
                nmpc_params["w_max"],
                nmpc_params["w_max"],
                nmpc_params["w_max"],
            ]
        )

        # # bu
        ocp.constraints.idxbu = np.array([0, 1, 2, 3])
        ocp.constraints.lbu = np.array(
            [nmpc_params["thrust_min"], nmpc_params["thrust_min"], nmpc_params["thrust_min"], nmpc_params["thrust_min"]]
        )
        ocp.constraints.ubu = np.array(
            [nmpc_params["thrust_max"], nmpc_params["thrust_max"], nmpc_params["thrust_max"], nmpc_params["thrust_max"]]
        )

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
        ocp.solver_options.qp_solver_cond_N = nmpc_params["N_node"]
        ocp.solver_options.tf = nmpc_params["T_pred"]

        # compile acados ocp
        json_file_path = os.path.join("./" + opt_model.name + "_acados_ocp.json")
        self.solver = AcadosOcpSolver(ocp, json_file=json_file_path, build=True)


class BeetleFullModel(object):
    def __init__(self):
        model_name = "beetle_full_model"

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

        wx = ca.SX.sym("wx")
        wy = ca.SX.sym("wy")
        wz = ca.SX.sym("wz")

        states = ca.vertcat(x, y, z, vx, vy, vz, qw, qx, qy, qz, wx, wy, wz)

        # parameters
        qwr = ca.SX.sym("qwr")  # reference for quaternions
        qxr = ca.SX.sym("qxr")
        qyr = ca.SX.sym("qyr")
        qzr = ca.SX.sym("qzr")
        parameters = ca.vertcat(qwr, qxr, qyr, qzr)

        # control inputs
        ft1 = ca.SX.sym("ft1")
        ft2 = ca.SX.sym("ft2")
        ft3 = ca.SX.sym("ft3")
        ft4 = ca.SX.sym("ft4")
        controls = ca.vertcat(ft1, ft2, ft3, ft4)

        c = (ft1 + ft2 + ft3 + ft4) / mass

        # dynamic model
        ds = ca.vertcat(
            vx,
            vy,
            vz,
            2 * (qx * qz + qw * qy) * c,
            2 * (qy * qz - qw * qx) * c,
            (1 - 2 * qx**2 - 2 * qy**2) * c - gravity,
            (-wx * qx - wy * qy - wz * qz) / 2,
            (wx * qw + wz * qy - wy * qz) / 2,
            (wy * qw - wz * qx + wx * qz) / 2,
            (wz * qw + wy * qx - wx * qy) / 2,
            (Iyy * wy * wz - Izz * wy * wz) / Ixx + (ft1 * p1[1] + ft2 * p2[1] + ft3 * p3[1] + ft4 * p4[1]) / Ixx,
            (-Ixx * wx * wz + Izz * wx * wz) / Iyy + (-ft1 * p1[0] - ft2 * p2[0] - ft3 * p3[0] - ft4 * p4[0]) / Iyy,
            (Ixx * wx * wy - Iyy * wx * wy) / Izz + (-dr1 * ft1 - dr2 * ft2 - dr3 * ft3 - dr4 * ft4) * kq_d_kt / Izz,
        )

        # function
        f = ca.Function("f", [states, controls], [ds], ["state", "control_input"], ["ds"], {"allow_free": True})

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
            wx,
            wy,
            wz,
        )
        control_y = controls

        # acados model
        x_dot = ca.SX.sym("x_dot", 13)
        f_impl = x_dot - f(states, controls)

        model = AcadosModel()
        model.name = model_name
        model.f_expl_expr = f(states, controls)  # CasADi expression for the explicit dynamics
        model.f_impl_expr = f_impl  # CasADi expression for the implicit dynamics
        model.x = states
        model.xdot = x_dot
        model.u = controls
        model.p = parameters
        model.cost_y_expr = ca.vertcat(state_y, control_y)  # NONLINEAR_LS
        model.cost_y_expr_e = state_y

        self.model = model


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


if __name__ == "__main__":
    # read parameters from launch file
    mpc_ctl = NMPCController()

    print("Successfully initialized acados ocp solver: ", mpc_ctl.solver)
    print("T_samp: ", nmpc_params["T_samp"])
    print("T_pred: ", nmpc_params["T_pred"])
    print("T_integ: ", nmpc_params["T_integ"])
    print("N_node: ", nmpc_params["N_node"])
