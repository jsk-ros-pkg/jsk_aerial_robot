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
param_path = os.path.join(rospack.get_path("beetle"), "config", "BeetleNMPCNoServoDelay_sim.yaml")
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
p1_b = physical_params["p1"]
dr2 = physical_params["dr2"]
p2_b = physical_params["p2"]
dr3 = physical_params["dr3"]
p3_b = physical_params["p3"]
dr4 = physical_params["dr4"]
p4_b = physical_params["p4"]
kq_d_kt = physical_params["kq_d_kt"]

x_init = np.zeros(13)  # TODO: make the number of states adjustable
x_init[6] = 1.0  # qw
u_init = np.zeros(8)
u_init[0:4] = mass * gravity / 4  # ft1, ft2, ft3, ft4


def create_acados_ocp() -> AcadosOcp:
    ocp_model = create_acados_model()

    nx = ocp_model.x.size()[0]
    nu = ocp_model.u.size()[0]
    n_params = ocp_model.p.size()[0]

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
    ocp.model = ocp_model
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
    print("Q: \n", Q)

    R = np.diag(
        [
            nmpc_params["Rt"],
            nmpc_params["Rt"],
            nmpc_params["Rt"],
            nmpc_params["Rt"],
            nmpc_params["Rac"],
            nmpc_params["Rac"],
            nmpc_params["Rac"],
            nmpc_params["Rac"],
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
    ocp.constraints.idxbx = np.array([3, 4, 5, 10, 11, 12])
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
    print("lbx: ", ocp.constraints.lbx)
    print("ubx: ", ocp.constraints.ubx)

    # # bx_e
    # vx, vy, vz, wx, wy, wz, a1, a2, a3, a4
    ocp.constraints.idxbx_e = np.array([3, 4, 5, 10, 11, 12])
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
    print("lbx_e: ", ocp.constraints.lbx_e)
    print("ubx_e: ", ocp.constraints.ubx_e)

    # # bu
    # ft1, ft2, ft3, ft4, a1c, a2c, a3c, a4c
    ocp.constraints.idxbu = np.array([0, 1, 2, 3, 4, 5, 6, 7])
    ocp.constraints.lbu = np.array(
        [
            nmpc_params["thrust_min"],
            nmpc_params["thrust_min"],
            nmpc_params["thrust_min"],
            nmpc_params["thrust_min"],
            nmpc_params["a_min"],
            nmpc_params["a_min"],
            nmpc_params["a_min"],
            nmpc_params["a_min"],
        ]
    )
    ocp.constraints.ubu = np.array(
        [
            nmpc_params["thrust_max"],
            nmpc_params["thrust_max"],
            nmpc_params["thrust_max"],
            nmpc_params["thrust_max"],
            nmpc_params["a_max"],
            nmpc_params["a_max"],
            nmpc_params["a_max"],
            nmpc_params["a_max"],
        ]
    )
    print("lbu: ", ocp.constraints.lbu)
    print("ubu: ", ocp.constraints.ubu)

    # initial state
    ocp.constraints.x0 = x_init
    ocp.cost.yref = np.concatenate((x_init, u_init))
    ocp.cost.yref_e = x_init

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
    solver = AcadosOcpSolver(ocp, json_file=json_file_path, build=True)

    return ocp


def create_acados_model() -> AcadosModel:
    model_name = "beetle_no_servo_delay_model"

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

    states = ca.vertcat(p, v, q, w)

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
    ft = ca.vertcat(ft1, ft2, ft3, ft4)
    a1 = ca.SX.sym("a1")
    a2 = ca.SX.sym("a2")
    a3 = ca.SX.sym("a3")
    a4 = ca.SX.sym("a4")
    ac = ca.vertcat(a1, a2, a3, a4)
    controls = ca.vertcat(ft, ac)

    # transformation matrix
    row_1 = ca.horzcat(
        ca.SX(1 - 2 * qy**2 - 2 * qz**2), ca.SX(2 * qx * qy - 2 * qw * qz), ca.SX(2 * qx * qz + 2 * qw * qy)
    )
    row_2 = ca.horzcat(
        ca.SX(2 * qx * qy + 2 * qw * qz), ca.SX(1 - 2 * qx**2 - 2 * qz**2), ca.SX(2 * qy * qz - 2 * qw * qx)
    )
    row_3 = ca.horzcat(
        ca.SX(2 * qx * qz - 2 * qw * qy), ca.SX(2 * qy * qz + 2 * qw * qx), ca.SX(1 - 2 * qx**2 - 2 * qy**2)
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
        ca.mtimes(rot_ib, f_u_b) / mass + g_i,
        (-wx * qx - wy * qy - wz * qz) / 2,
        (wx * qw + wz * qy - wy * qz) / 2,
        (wy * qw - wz * qx + wx * qz) / 2,
        (wz * qw + wy * qx - wx * qy) / 2,
        ca.mtimes(inv_iv, (-ca.cross(w, ca.mtimes(iv, w)) + tau_u_b)),
    )

    # function
    func = ca.Function("func", [states, controls], [ds], ["state", "control_input"], ["ds"])

    # NONLINEAR_LS = error^T @ Q @ error; error = y - y_ref
    qe_x = qwr * qx - qw * qxr + qyr * qz - qy * qzr
    qe_y = qwr * qy - qw * qyr - qxr * qz + qx * qzr
    qe_z = qxr * qy - qx * qyr + qwr * qz - qw * qzr

    state_y = ca.vertcat(p, v, qwr, qe_x + qxr, qe_y + qyr, qe_z + qzr, w)
    control_y = controls

    # acados model
    x_dot = ca.SX.sym("x_dot", 13)
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
    model.cost_y_expr_e = state_y

    return model


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


def closed_loop_simulation(ocp: AcadosOcp):
    acados_ocp_solver = AcadosOcpSolver(ocp, json_file="acados_ocp_" + ocp.model.name + ".json")
    acados_integrator = AcadosSimSolver(ocp, json_file="acados_ocp_" + ocp.model.name + ".json")
    # TODO: change to AcadosSim

    N_sim = 100
    nx = ocp.model.x.size()[0]
    nu = ocp.model.u.size()[0]

    x_sim_all = np.ndarray((N_sim + 1, nx))
    u_sim_all = np.ndarray((N_sim, nu))

    x_current = x_init
    x_sim_all[0, :] = x_current

    # ============ initialize =============
    for stage in range(acados_ocp_solver.N + 1):
        acados_ocp_solver.set(stage, "x", x_init)
    for stage in range(acados_ocp_solver.N):
        acados_ocp_solver.set(stage, "u", u_init)

    # ============ update =============
    # get x and u, set reference
    xr = np.zeros([acados_ocp_solver.N + 1, 13])
    xr[:, 6] = 1.0  # qw
    xr[:, 2] = 1.0  # z
    ur = np.zeros([acados_ocp_solver.N, 8])
    ur[:, 0:4] = mass * gravity / 4  # ft1, ft2, ft3, ft4   # TODO: consider remove this line

    for i in range(N_sim):

        # -------- update solver --------
        for j in range(acados_ocp_solver.N):
            yr = np.concatenate((xr[j, :], ur[j, :]))
            acados_ocp_solver.set(j, "yref", yr)

            quaternion_r = xr[j, 6:10]
            acados_ocp_solver.set(j, "p", quaternion_r)  # for nonlinear quaternion error
        acados_ocp_solver.set(acados_ocp_solver.N, "yref", xr[acados_ocp_solver.N, :])  # final state of x, no u

        quaternion_r = xr[acados_ocp_solver.N, 6:10]
        acados_ocp_solver.set(acados_ocp_solver.N, "p", quaternion_r)  # for nonlinear quaternion error

        # feedback, take the first action
        u0 = acados_ocp_solver.solve_for_x0(x_current)
        if acados_ocp_solver.status != 0:
            raise Exception("acados acados_ocp_solver returned status {}. Exiting.".format(acados_ocp_solver.status))
        u_sim_all[i, :] = u0

        # --------- update simulation ----------
        acados_integrator.set("x", x_current)
        acados_integrator.set("u", u0)
        status = acados_integrator.solve()
        if status != 0:
            raise Exception(f"acados integrator returned status {status} in closed loop instance {i}")

        # update state
        x_current = acados_integrator.get("x")
        x_sim_all[i + 1, :] = x_current

    # plot
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(10, 10))
    fig.suptitle(f"NMPC closed-loop sim no servo delay with servo angle limit {nmpc_params['a_max']} rad")

    plt.subplot(3, 1, 1)
    plt.plot(np.arange(x_sim_all.shape[0]) * nmpc_params["T_samp"], x_sim_all[:, 0], label="x")
    plt.plot(np.arange(x_sim_all.shape[0]) * nmpc_params["T_samp"], x_sim_all[:, 1], label="y")
    plt.plot(np.arange(x_sim_all.shape[0]) * nmpc_params["T_samp"], x_sim_all[:, 2], label="z")
    plt.legend()
    plt.xlabel("time (s)")
    plt.ylabel("position (m)")
    plt.grid(True)

    plt.subplot(3, 1, 2)
    plt.plot(np.arange(u_sim_all.shape[0]) * nmpc_params["T_samp"], u_sim_all[:, 0], label="ft1")
    plt.plot(np.arange(u_sim_all.shape[0]) * nmpc_params["T_samp"], u_sim_all[:, 1], label="ft2")
    plt.plot(np.arange(u_sim_all.shape[0]) * nmpc_params["T_samp"], u_sim_all[:, 2], label="ft3")
    plt.plot(np.arange(u_sim_all.shape[0]) * nmpc_params["T_samp"], u_sim_all[:, 3], label="ft4")
    plt.legend()
    plt.xlabel("time (s)")
    plt.ylabel("thrust (N)")
    plt.grid(True)

    plt.subplot(3, 1, 3)
    plt.plot(np.arange(u_sim_all.shape[0]) * nmpc_params["T_samp"], u_sim_all[:, 4], label="a1")
    plt.plot(np.arange(u_sim_all.shape[0]) * nmpc_params["T_samp"], u_sim_all[:, 5], label="a2")
    plt.plot(np.arange(u_sim_all.shape[0]) * nmpc_params["T_samp"], u_sim_all[:, 6], label="a3")
    plt.plot(np.arange(u_sim_all.shape[0]) * nmpc_params["T_samp"], u_sim_all[:, 7], label="a4")
    plt.legend()
    plt.xlabel("time (s)")
    plt.ylabel("servo angle (rad)")
    plt.grid(True)

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])

    plt.show()


if __name__ == "__main__":
    # read parameters from launch file
    acados_ocp = create_acados_ocp()
    print("Successfully initialized acados ocp: ", acados_ocp)
    print("number of states: ", acados_ocp.dims.nx)
    print("number of controls: ", acados_ocp.dims.nu)
    print("number of parameters: ", acados_ocp.dims.np)
    print("T_samp: ", nmpc_params["T_samp"])
    print("T_pred: ", nmpc_params["T_pred"])
    print("T_integ: ", nmpc_params["T_integ"])
    print("N_node: ", nmpc_params["N_node"])

    closed_loop_simulation(acados_ocp)
