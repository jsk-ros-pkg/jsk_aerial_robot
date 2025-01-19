#!/usr/bin/env python
# -*- encoding: ascii -*-
"""
Author: LI Jinjie
File: tilt_qd_servo_dist_imp.py
Date: 2025/01/15 21:58 PM
Description: impedance control.
"""
from __future__ import print_function  # be compatible with python2
import os
import sys
import numpy as np
import yaml
import rospkg
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
from tf_conversions import transformations as tf
import casadi as ca

from nmpc_base import NMPCBase, XrUrConverterBase

from phys_param_beetle_omni import *

# read parameters from yaml
rospack = rospkg.RosPack()

nmpc_param_path = os.path.join(rospack.get_path("beetle_omni"), "config", "BeetleNMPCFullServoImp.yaml")
with open(nmpc_param_path, "r") as f:
    nmpc_param_dict = yaml.load(f, Loader=yaml.FullLoader)
nmpc_params = nmpc_param_dict["controller"]["nmpc"]
nmpc_params["N_node"] = int(nmpc_params["T_pred"] / nmpc_params["T_integ"])

pM_imp = np.diag([nmpc_params["pMxy"], nmpc_params["pMxy"], nmpc_params["pMz"]])
pD_imp = np.diag([nmpc_params["Qv_xy"], nmpc_params["Qv_xy"], nmpc_params["Qv_z"]])
pK_imp = np.diag([nmpc_params["Qp_xy"], nmpc_params["Qp_xy"], nmpc_params["Qp_z"]])

oM_imp = np.diag([nmpc_params["oMxy"], nmpc_params["oMxy"], nmpc_params["oMz"]])
oD_imp = np.diag([nmpc_params["Qw_xy"], nmpc_params["Qw_xy"], nmpc_params["Qw_z"]])
oK_imp = np.diag([nmpc_params["Qq_xy"], nmpc_params["Qq_xy"], nmpc_params["Qq_z"]])


class NMPCTiltQdServoImpedance(NMPCBase):
    def __init__(self):
        super(NMPCTiltQdServoImpedance, self).__init__()
        self.t_servo = t_servo

        self.fake_sensor = FakeSensor()

    def set_name(self) -> str:
        return "tilt_qd_servo_dist_imp_mdl"

    def set_ts_ctrl(self) -> float:
        return nmpc_params["T_samp"]

    def create_acados_model(self, model_name: str) -> AcadosModel:
        # model states
        p = ca.SX.sym("p", 3)
        v = ca.SX.sym("v", 3)

        qw = ca.SX.sym("qw")
        qx = ca.SX.sym("qx")
        qy = ca.SX.sym("qy")
        qz = ca.SX.sym("qz")
        q = ca.vertcat(qw, qx, qy, qz)

        wx = ca.SX.sym("wx")
        wy = ca.SX.sym("wy")
        wz = ca.SX.sym("wz")
        w = ca.vertcat(wx, wy, wz)

        a1 = ca.SX.sym("a1")
        a2 = ca.SX.sym("a2")
        a3 = ca.SX.sym("a3")
        a4 = ca.SX.sym("a4")
        a = ca.vertcat(a1, a2, a3, a4)

        f_d_i = ca.SX.sym("fd", 3)  # world frame
        tau_d_b = ca.SX.sym("tau_d", 3)  # body frame

        states = ca.vertcat(p, v, q, w, a, f_d_i, tau_d_b)

        # parameters
        qwr = ca.SX.sym("qwr")  # reference for quaternions
        qxr = ca.SX.sym("qxr")
        qyr = ca.SX.sym("qyr")
        qzr = ca.SX.sym("qzr")
        qr = ca.vertcat(qwr, qxr, qyr, qzr)

        mpx = ca.SX.sym("mpx")  # virtual mass
        mpy = ca.SX.sym("mpy")
        mpz = ca.SX.sym("mpz")
        mp = ca.vertcat(mpx, mpy, mpz)

        mqx = ca.SX.sym("mqx")  # virtual inertia
        mqy = ca.SX.sym("mqy")
        mqz = ca.SX.sym("mqz")
        mq = ca.vertcat(mqx, mqy, mqz)

        f_d_i_para = ca.SX.sym("fd_para", 3)  # world frame
        tau_d_b_para = ca.SX.sym("tau_d_para", 3)  # body frame

        parameters = ca.vertcat(qr, f_d_i_para, tau_d_b_para, mp, mq)

        # control inputs
        ft1 = ca.SX.sym("ft1")
        ft2 = ca.SX.sym("ft2")
        ft3 = ca.SX.sym("ft3")
        ft4 = ca.SX.sym("ft4")
        ft = ca.vertcat(ft1, ft2, ft3, ft4)

        a1c = ca.SX.sym("a1c")
        a2c = ca.SX.sym("a2c")
        a3c = ca.SX.sym("a3c")
        a4c = ca.SX.sym("a4c")
        ac = ca.vertcat(a1c, a2c, a3c, a4c)

        controls = ca.vertcat(ft, ac)

        # transformation matrix
        row_1 = ca.horzcat(
            ca.SX(1 - 2 * qy ** 2 - 2 * qz ** 2), ca.SX(2 * qx * qy - 2 * qw * qz), ca.SX(2 * qx * qz + 2 * qw * qy)
        )
        row_2 = ca.horzcat(
            ca.SX(2 * qx * qy + 2 * qw * qz), ca.SX(1 - 2 * qx ** 2 - 2 * qz ** 2), ca.SX(2 * qy * qz - 2 * qw * qx)
        )
        row_3 = ca.horzcat(
            ca.SX(2 * qx * qz - 2 * qw * qy), ca.SX(2 * qy * qz + 2 * qw * qx), ca.SX(1 - 2 * qx ** 2 - 2 * qy ** 2)
        )
        rot_ib = ca.vertcat(row_1, row_2, row_3)

        den = np.sqrt(p1_b[0] ** 2 + p1_b[1] ** 2)
        rot_be1 = np.array([[p1_b[0] / den, -p1_b[1] / den, 0], [p1_b[1] / den, p1_b[0] / den, 0], [0, 0, 1]])

        den = np.sqrt(p2_b[0] ** 2 + p2_b[1] ** 2)
        rot_be2 = np.array([[p2_b[0] / den, -p2_b[1] / den, 0], [p2_b[1] / den, p2_b[0] / den, 0], [0, 0, 1]])

        den = np.sqrt(p3_b[0] ** 2 + p3_b[1] ** 2)
        rot_be3 = np.array([[p3_b[0] / den, -p3_b[1] / den, 0], [p3_b[1] / den, p3_b[0] / den, 0], [0, 0, 1]])

        den = np.sqrt(p4_b[0] ** 2 + p4_b[1] ** 2)
        rot_be4 = np.array([[p4_b[0] / den, -p4_b[1] / den, 0], [p4_b[1] / den, p4_b[0] / den, 0], [0, 0, 1]])

        rot_e1r1 = ca.vertcat(
            ca.horzcat(1, 0, 0), ca.horzcat(0, ca.cos(a1), -ca.sin(a1)), ca.horzcat(0, ca.sin(a1), ca.cos(a1))
        )
        rot_e2r2 = ca.vertcat(
            ca.horzcat(1, 0, 0), ca.horzcat(0, ca.cos(a2), -ca.sin(a2)), ca.horzcat(0, ca.sin(a2), ca.cos(a2))
        )
        rot_e3r3 = ca.vertcat(
            ca.horzcat(1, 0, 0), ca.horzcat(0, ca.cos(a3), -ca.sin(a3)), ca.horzcat(0, ca.sin(a3), ca.cos(a3))
        )
        rot_e4r4 = ca.vertcat(
            ca.horzcat(1, 0, 0), ca.horzcat(0, ca.cos(a4), -ca.sin(a4)), ca.horzcat(0, ca.sin(a4), ca.cos(a4))
        )

        # inertial
        iv = ca.diag([Ixx, Iyy, Izz])
        inv_iv = ca.diag([1 / Ixx, 1 / Iyy, 1 / Izz])
        g_i = np.array([0, 0, -gravity])

        # wrench
        ft_r1 = ca.vertcat(0, 0, ft1)
        ft_r2 = ca.vertcat(0, 0, ft2)
        ft_r3 = ca.vertcat(0, 0, ft3)
        ft_r4 = ca.vertcat(0, 0, ft4)

        tau_r1 = ca.vertcat(0, 0, -dr1 * ft1 * kq_d_kt)
        tau_r2 = ca.vertcat(0, 0, -dr2 * ft2 * kq_d_kt)
        tau_r3 = ca.vertcat(0, 0, -dr3 * ft3 * kq_d_kt)
        tau_r4 = ca.vertcat(0, 0, -dr4 * ft4 * kq_d_kt)

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

        # dynamic model
        ds = ca.vertcat(
            v,
            (ca.mtimes(rot_ib, f_u_b) + f_d_i + f_d_i_para) / mass + g_i,
            (-wx * qx - wy * qy - wz * qz) / 2,
            (wx * qw + wz * qy - wy * qz) / 2,
            (wy * qw - wz * qx + wx * qz) / 2,
            (wz * qw + wy * qx - wx * qy) / 2,
            ca.mtimes(inv_iv, (-ca.cross(w, ca.mtimes(iv, w)) + tau_u_b + tau_d_b + tau_d_b_para)),
            (ac - a) / t_servo,
            ca.vertcat(0.0, 0.0, 0.0),
            ca.vertcat(0.0, 0.0, 0.0),
        )

        # Note that this part should be f_d_i and no f_d_i_para, since the impedance should not respond to the I Term force.
        lin_a_i = (ca.mtimes(rot_ib, f_u_b) + f_d_i + f_d_i_para) / mass + g_i
        ang_a_b = ca.mtimes(inv_iv, (-ca.cross(w, ca.mtimes(iv, w)) + tau_u_b + tau_d_b + tau_d_b_para))

        # function
        func = ca.Function("func", [states, controls], [ds], ["state", "control_input"], ["ds"], {"allow_free": True})

        # NONLINEAR_LS = error^T @ Q @ error; error = y - y_ref
        # qe = qr^* multiply q
        qe_w = qw * qwr + qx * qxr + qy * qyr + qz * qzr
        qe_x = qwr * qx - qw * qxr - qyr * qz + qy * qzr
        qe_y = qwr * qy - qw * qyr + qxr * qz - qx * qzr
        qe_z = -qxr * qy + qx * qyr + qwr * qz - qw * qzr

        p_mtx_imp_inv = ca.SX.zeros(3, 3)
        p_mtx_imp_inv[0, 0] = 1 / mpx
        p_mtx_imp_inv[1, 1] = 1 / mpy
        p_mtx_imp_inv[2, 2] = 1 / mpz

        o_mtx_imp_inv = ca.SX.zeros(3, 3)
        o_mtx_imp_inv[0, 0] = 1 / mqx
        o_mtx_imp_inv[1, 1] = 1 / mqy
        o_mtx_imp_inv[2, 2] = 1 / mqz

        state_y = ca.vertcat(p, v, qwr, qe_x + qxr, qe_y + qyr, qe_z + qzr, w, a,
                             lin_a_i - ca.mtimes(p_mtx_imp_inv, f_d_i), ang_a_b - ca.mtimes(o_mtx_imp_inv, tau_d_b))
        state_y_e = ca.vertcat(p, v, qwr, qe_x + qxr, qe_y + qyr, qe_z + qzr, w, a,
                               ca.vertcat(0, 0, 0), ca.vertcat(0, 0, 0))  # acc = 0 for infinite horizon
        control_y = ca.vertcat(ft, (ac - a))  # ac_ref must be zero!

        # acados model
        x_dot = ca.SX.sym("x_dot", states.size()[0])
        f_impl = x_dot - func(states, controls)

        model = AcadosModel()
        model.name = model_name
        model.f_expl_expr = func(states, controls)  # CasADi expression for the explicit dynamics
        model.f_impl_expr = f_impl  # CasADi expression for the implicit dynamics
        model.x = states
        model.xdot = x_dot
        model.u = controls
        model.p = parameters
        model.cost_y_expr = ca.vertcat(state_y, control_y)  # NONLINEAR_LS
        model.cost_y_expr_e = state_y_e

        return model

    def create_acados_ocp_solver(self, ocp_model: AcadosModel, is_build: bool) -> AcadosOcpSolver:
        nx = ocp_model.x.size()[0]
        nu = ocp_model.u.size()[0]
        n_params = ocp_model.p.size()[0]

        # get file path for acados
        rospack = rospkg.RosPack()
        folder_path = os.path.join(
            rospack.get_path("aerial_robot_control"), "include", "aerial_robot_control", "nmpc", ocp_model.name
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
        ocp.dims.N = nmpc_params["N_node"]

        # initialize parameters
        ocp.dims.np = n_params
        ocp.parameter_values = np.zeros(n_params)

        # cost function
        # see https://docs.acados.org/python_interface/#acados_template.acados_ocp.AcadosOcpCost for details
        Q = np.diag(
            [
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                nmpc_params["Qa"],
                nmpc_params["Qa"],
                nmpc_params["Qa"],
                nmpc_params["Qa"],
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            ]
        )

        p_weight = np.concatenate([pK_imp, pD_imp, pM_imp])
        p_weight_mtx = np.dot(p_weight, p_weight.T)
        Q[0:6, 0:6] = p_weight_mtx[0:6, 0:6]
        Q[17:20, 17:20] = p_weight_mtx[6:9, 6:9]
        Q[0:6, 17:20] = p_weight_mtx[0:6, 6:9]
        Q[17:20, 0:6] = p_weight_mtx[6:9, 0:6]

        o_weight = np.concatenate([oK_imp, oD_imp, oM_imp])
        o_weight_mtx = np.dot(o_weight, o_weight.T)
        Q[7:13, 7:13] = o_weight_mtx[0:6, 0:6]
        Q[20:23, 20:23] = o_weight_mtx[6:9, 6:9]
        Q[7:13, 20:23] = o_weight_mtx[0:6, 6:9]
        Q[20:23, 7:13] = o_weight_mtx[6:9, 0:6]

        print("Q: \n", Q)

        R = np.diag(
            [
                nmpc_params["Rt"],
                nmpc_params["Rt"],
                nmpc_params["Rt"],
                nmpc_params["Rt"],
                nmpc_params["Rac_d"],
                nmpc_params["Rac_d"],
                nmpc_params["Rac_d"],
                nmpc_params["Rac_d"],
            ]
        )
        print("R: \n", R)

        ocp.cost.cost_type = "NONLINEAR_LS"
        ocp.cost.cost_type_e = "NONLINEAR_LS"
        ocp.cost.W = np.block([[Q, np.zeros((nx, nu))], [np.zeros((nu, nx)), R]])
        ocp.cost.W_e = Q  # weight matrix at terminal shooting node (N).

        # set constraints
        # # bx
        # vx, vy, vz, wx, wy, wz, a1, a2, a3, a4
        ocp.constraints.idxbx = np.array([3, 4, 5, 10, 11, 12, 13, 14, 15, 16])
        ocp.constraints.lbx = np.array(
            [
                nmpc_params["v_min"],
                nmpc_params["v_min"],
                nmpc_params["v_min"],
                nmpc_params["w_min"],
                nmpc_params["w_min"],
                nmpc_params["w_min"],
                nmpc_params["a_min"],
                nmpc_params["a_min"],
                nmpc_params["a_min"],
                nmpc_params["a_min"],
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
                nmpc_params["a_max"],
                nmpc_params["a_max"],
                nmpc_params["a_max"],
                nmpc_params["a_max"],
            ]
        )
        print("lbx: ", ocp.constraints.lbx)
        print("ubx: ", ocp.constraints.ubx)

        # # bx_e
        # vx, vy, vz, wx, wy, wz, a1, a2, a3, a4
        ocp.constraints.idxbx_e = np.array([3, 4, 5, 10, 11, 12, 13, 14, 15, 16])
        ocp.constraints.lbx_e = np.array(
            [
                nmpc_params["v_min"],
                nmpc_params["v_min"],
                nmpc_params["v_min"],
                nmpc_params["w_min"],
                nmpc_params["w_min"],
                nmpc_params["w_min"],
                nmpc_params["a_min"],
                nmpc_params["a_min"],
                nmpc_params["a_min"],
                nmpc_params["a_min"],
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
                nmpc_params["a_max"],
                nmpc_params["a_max"],
                nmpc_params["a_max"],
                nmpc_params["a_max"],
            ]
        )
        print("lbx_e: ", ocp.constraints.lbx_e)
        print("ubx_e: ", ocp.constraints.ubx_e)

        # # bu
        # ft1, ft2, ft3, ft4
        ocp.constraints.idxbu = np.array([0, 1, 2, 3])
        ocp.constraints.lbu = np.array(
            [
                nmpc_params["thrust_min"],
                nmpc_params["thrust_min"],
                nmpc_params["thrust_min"],
                nmpc_params["thrust_min"],
            ]
        )
        ocp.constraints.ubu = np.array(
            [
                nmpc_params["thrust_max"],
                nmpc_params["thrust_max"],
                nmpc_params["thrust_max"],
                nmpc_params["thrust_max"],
            ]
        )
        print("lbu: ", ocp.constraints.lbu)
        print("ubu: ", ocp.constraints.ubu)

        # initial state
        x_ref = np.zeros(nx)
        x_ref[6] = 1.0  # qw
        u_ref = np.zeros(nu)
        u_ref[0:4] = mass * gravity / 4  # ft1, ft2, ft3, ft4
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
        json_file_path = os.path.join("./" + ocp_model.name + "_acados_ocp.json")
        solver = AcadosOcpSolver(ocp, json_file=json_file_path, build=is_build)

        return solver

    def create_xr_ur_converter(self) -> XrUrConverterBase:
        return XrUrConverter()


class XrUrConverter(XrUrConverterBase):
    def __init__(self):
        super(XrUrConverter, self).__init__()

    def _set_nx_nu(self):
        self.nx = 23
        self.nu = 8

    def _set_physical_params(self):
        self.p1_b = p1_b
        self.p2_b = p2_b
        self.p3_b = p3_b
        self.p4_b = p4_b
        self.dr1 = dr1
        self.dr2 = dr2
        self.dr3 = dr3
        self.dr4 = dr4
        self.kq_d_kt = kq_d_kt

        self.mass = mass
        self.gravity = gravity

        self.alloc_mat = self._get_alloc_mat()
        self.alloc_mat_pinv = self._get_alloc_mat_pinv()
        self.ocp_N = nmpc_params["N_node"]

    def pose_point_2_xr_ur(self, target_xyz, target_rpy):
        roll = target_rpy.item(0)
        pitch = target_rpy.item(1)
        yaw = target_rpy.item(2)

        q = tf.quaternion_from_euler(roll, pitch, yaw, axes="sxyz")
        target_qwxyz = np.array([[q[3], q[0], q[1], q[2]]]).T

        # convert [0,0,gravity] to body frame
        q_inv = tf.quaternion_inverse(q)
        rot = tf.quaternion_matrix(q_inv)
        fg_i = np.array([0, 0, self.mass * self.gravity, 0])
        fg_b = rot @ fg_i
        target_wrench = np.array([[fg_b.item(0), fg_b.item(1), fg_b.item(2), 0, 0, 0]]).T

        # a quicker method if alloc_mat is dynamic:  x, _, _, _ = np.linalg.lstsq(alloc_mat, target_wrench, rcond=None)
        x = self.alloc_mat_pinv @ target_wrench

        a1_ref = np.arctan2(x[0, 0], x[1, 0])
        ft1_ref = np.sqrt(x[0, 0] ** 2 + x[1, 0] ** 2)
        a2_ref = np.arctan2(x[2, 0], x[3, 0])
        ft2_ref = np.sqrt(x[2, 0] ** 2 + x[3, 0] ** 2)
        a3_ref = np.arctan2(x[4, 0], x[5, 0])
        ft3_ref = np.sqrt(x[4, 0] ** 2 + x[5, 0] ** 2)
        a4_ref = np.arctan2(x[6, 0], x[7, 0])
        ft4_ref = np.sqrt(x[6, 0] ** 2 + x[7, 0] ** 2)

        # get x and u, set reference
        ocp_N = self.ocp_N

        xr = np.zeros([ocp_N + 1, self.nx])
        xr[:, 0] = target_xyz.item(0)  # x
        xr[:, 1] = target_xyz.item(1)  # y
        xr[:, 2] = target_xyz.item(2)  # z
        xr[:, 6] = target_qwxyz.item(0)  # qx
        xr[:, 7] = target_qwxyz.item(1)  # qx
        xr[:, 8] = target_qwxyz.item(2)  # qy
        xr[:, 9] = target_qwxyz.item(3)  # qz
        xr[:, 13] = a1_ref
        xr[:, 14] = a2_ref
        xr[:, 15] = a3_ref
        xr[:, 16] = a4_ref

        ur = np.zeros([ocp_N, self.nu])
        ur[:, 0] = ft1_ref
        ur[:, 1] = ft2_ref
        ur[:, 2] = ft3_ref
        ur[:, 3] = ft4_ref

        return xr, ur


class FakeSensor:
    def __init__(self):
        self.mass = mass
        self.gravity = gravity

        self.iv = np.diag([Ixx, Iyy, Izz])
        self.inv_iv = np.diag([1 / Ixx, 1 / Iyy, 1 / Izz])
        self.g_i = np.array([0, 0, -gravity])

        self.dr1 = dr1
        self.p1_b = p1_b
        self.dr2 = dr2
        self.p2_b = p2_b
        self.dr3 = dr3
        self.p3_b = p3_b
        self.dr4 = dr4
        self.p4_b = p4_b
        self.kq_d_kt = kq_d_kt

        den = np.sqrt(p1_b[0] ** 2 + p1_b[1] ** 2)
        self.rot_be1 = np.array([[p1_b[0] / den, -p1_b[1] / den, 0], [p1_b[1] / den, p1_b[0] / den, 0], [0, 0, 1]])
        den = np.sqrt(p2_b[0] ** 2 + p2_b[1] ** 2)
        self.rot_be2 = np.array([[p2_b[0] / den, -p2_b[1] / den, 0], [p2_b[1] / den, p2_b[0] / den, 0], [0, 0, 1]])
        den = np.sqrt(p3_b[0] ** 2 + p3_b[1] ** 2)
        self.rot_be3 = np.array([[p3_b[0] / den, -p3_b[1] / den, 0], [p3_b[1] / den, p3_b[0] / den, 0], [0, 0, 1]])
        den = np.sqrt(p4_b[0] ** 2 + p4_b[1] ** 2)
        self.rot_be4 = np.array([[p4_b[0] / den, -p4_b[1] / den, 0], [p4_b[1] / den, p4_b[0] / den, 0], [0, 0, 1]])

    def update_acc(self, x):
        qw, qx, qy, qz = x[6:10]
        w = x[10:13]
        a1, a2, a3, a4 = x[13:17]
        ft1, ft2, ft3, ft4 = x[17:21]
        f_d_i = x[21:24]
        tau_d_b = x[24:27]

        row_1 = np.array([1 - 2 * qy ** 2 - 2 * qz ** 2, 2 * qx * qy - 2 * qw * qz, 2 * qx * qz + 2 * qw * qy])
        row_2 = np.array([2 * qx * qy + 2 * qw * qz, 1 - 2 * qx ** 2 - 2 * qz ** 2, 2 * qy * qz - 2 * qw * qx])
        row_3 = np.array([2 * qx * qz - 2 * qw * qy, 2 * qy * qz + 2 * qw * qx, 1 - 2 * qx ** 2 - 2 * qy ** 2])
        rot_ib = np.vstack((row_1, row_2, row_3))
        rot_bi = rot_ib.T

        rot_e1r1 = self.rot_e2r(a1)
        rot_e2r2 = self.rot_e2r(a2)
        rot_e3r3 = self.rot_e2r(a3)
        rot_e4r4 = self.rot_e2r(a4)

        ft_r1 = np.array([0, 0, ft1])
        ft_r2 = np.array([0, 0, ft2])
        ft_r3 = np.array([0, 0, ft3])
        ft_r4 = np.array([0, 0, ft4])

        tau_r1 = np.array([0, 0, -self.dr1 * ft1 * self.kq_d_kt])
        tau_r2 = np.array([0, 0, -self.dr2 * ft2 * self.kq_d_kt])
        tau_r3 = np.array([0, 0, -self.dr3 * ft3 * self.kq_d_kt])
        tau_r4 = np.array([0, 0, -self.dr4 * ft4 * self.kq_d_kt])

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

        sf_b = (f_u_b + np.dot(rot_bi, f_d_i)) / self.mass  # specific force in body frame
        ang_acc_b = np.dot(self.inv_iv, (-np.cross(w, np.dot(self.iv, w)) + tau_u_b + tau_d_b))

        return sf_b, ang_acc_b, rot_ib

    @staticmethod
    def rot_e2r(a):
        return np.array([[1, 0, 0], [0, np.cos(a), -np.sin(a)], [0, np.sin(a), np.cos(a)]])


class FIRDifferentiator:
    def __init__(self, coefficients, sampling_frequency):
        """
        Initializes the FIR Differentiator with the provided filter coefficients and sampling frequency.

        :param coefficients: The FIR filter coefficients.
        :param sampling_frequency: The sampling frequency of the input signal in Hz.
        """
        self.coefficients = np.array(coefficients)
        self.sampling_frequency = sampling_frequency
        self.buffer = np.zeros(len(coefficients))

    def apply_single(self, sample):
        """
        Applies the FIR differentiator to a single input sample.

        :param sample: Input sample (a single number).
        :return: Filtered output (a single number).
        """
        # Shift the buffer and add the new signal sample
        self.buffer[1:] = self.buffer[:-1]
        self.buffer[0] = sample

        # Calculate the output as the dot product of coefficients and buffer
        output = np.dot(self.coefficients, self.buffer)

        # Scale the output by the sampling frequency
        output *= self.sampling_frequency

        return output


if __name__ == "__main__":
    nmpc = NMPCTiltQdServoImpedance()

    acados_ocp_solver = nmpc.get_ocp_solver()
    print("Successfully initialized acados ocp: ", acados_ocp_solver.acados_ocp)
    print("number of states: ", acados_ocp_solver.acados_ocp.dims.nx)
    print("number of controls: ", acados_ocp_solver.acados_ocp.dims.nu)
    print("number of parameters: ", acados_ocp_solver.acados_ocp.dims.np)
    print("T_samp: ", nmpc_params["T_samp"])
    print("T_pred: ", nmpc_params["T_pred"])
    print("T_integ: ", nmpc_params["T_integ"])
    print("N_node: ", nmpc_params["N_node"])
