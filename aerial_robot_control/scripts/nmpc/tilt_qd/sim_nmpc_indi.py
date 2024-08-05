"""
 Created by li-jinjie on 24-8-5.
"""
import copy
import time

import numpy as np
import argparse
from acados_template import AcadosModel, AcadosSim, AcadosSimSolver

from nmpc_viz import Visualizer

from tilt_qd_servo_thrust_dist import NMPCTiltQdServoThrustDist

if __name__ == "__main__":
    # read arguments
    parser = argparse.ArgumentParser(description="Run the simulation of different disturbance rejection methods.")
    parser.add_argument(
        "dist_rej",
        type=int,
        help="The NMPC model to be simulated. Options: 0 (no disturbance rejection), 1 (INDI).",
    )
    parser.add_argument("-p", "--plot_type", type=int, default=0, help="The type of plot. Options: 0 (full), 1, 2.")

    args = parser.parse_args()

    # ========== init ==========
    # ---------- Controller ----------
    nmpc = NMPCTiltQdServoThrustDist()
    t_servo_ctrl = getattr(nmpc, "t_servo", 0.0)
    ts_ctrl = nmpc.ts_ctrl

    # ocp solver
    ocp_solver = nmpc.get_ocp_solver()
    nx = ocp_solver.acados_ocp.dims.nx
    nu = ocp_solver.acados_ocp.dims.nu
    n_param = ocp_solver.acados_ocp.dims.np  # np has been used for numpy

    x_init = np.zeros(nx)
    x_init[6] = 1.0  # qw
    u_init = np.zeros(nu)

    for stage in range(ocp_solver.N + 1):
        ocp_solver.set(stage, "x", x_init)
    for stage in range(ocp_solver.N):
        ocp_solver.set(stage, "u", u_init)

    # ---------- Simulator ----------
    sim_nmpc = NMPCTiltQdServoThrustDist()
    t_servo_sim = getattr(sim_nmpc, "t_servo", 0.0)

    ts_sim = 0.005

    disturbance = np.zeros(6)
    disturbance[2] = 1.0  # N, fz

    t_total_sim = 3.0
    if args.plot_type == 1:
        t_total_sim = 4.0
    if args.plot_type == 2:
        t_total_sim = 3.0

    N_sim = int(t_total_sim / ts_sim)

    # sim solver
    sim_nmpc.get_ocp_model()
    sim_solver = sim_nmpc.create_acados_sim_solver(sim_nmpc.get_ocp_model(), ts_sim, True)
    nx_sim = sim_solver.acados_sim.dims.nx

    x_init_sim = np.zeros(nx_sim)
    x_init_sim[6] = 1.0  # qw
    x_init_sim[-6:] = disturbance

    # ---------- Others ----------
    xr_ur_converter = nmpc.get_xr_ur_converter()
    viz = Visualizer(N_sim, nx_sim, nu, x_init_sim, is_record_diff_u=True)

    is_sqp_change = False
    t_sqp_start = 2.5
    t_sqp_end = 3.0

    # ========== update ==========
    u_cmd = u_init
    t_ctl = 0.0
    x_now_sim = x_init_sim
    for i in range(N_sim):
        # --------- update time ---------
        t_now = i * ts_sim
        t_ctl += ts_sim

        # --------- update state estimation ---------
        if isinstance(nmpc, NMPCTiltQdServoThrustDist):
            x_now = np.zeros(nx)
            x_now[:nx - 6] = x_now_sim[:nx - 6]  # copy elements except the last 6 elements, which are the disturbance
        else:
            x_now = x_now_sim[:nx]  # the dimension of x_now may be smaller than x_now_sim

        # -------- update control target --------
        target_xyz = np.array([[0.0, 0.0, 1.0]]).T
        target_rpy = np.array([[0.0, 0.0, 0.0]]).T

        if args.plot_type == 2:
            target_xyz = np.array([[0.0, 0.0, 0.0]]).T
            target_rpy = np.array([[0.5, 0.5, 0.5]]).T

        # if t_total_sim > 2.0:
        #     if 2.0 <= t_now < 6:
        #         target_xyz = np.array([[0.3, 0.6, 1.0]]).T
        #
        #         roll = 30.0 / 180.0 * np.pi
        #         pitch = 60.0 / 180.0 * np.pi
        #         yaw = 90.0 / 180.0 * np.pi
        #         target_rpy = np.array([[roll, pitch, yaw]]).T
        #
        #     if t_now >= 6:
        #         assert t_sqp_end <= 3.0
        #         target_xyz = np.array([[1.0, 1.0, 1.0]]).T
        #         target_rpy = np.array([[0.0, 0.0, 0.0]]).T

        xr, ur = xr_ur_converter.pose_point_2_xr_ur(target_xyz, target_rpy)

        if args.plot_type == 2:
            if nx > 13:
                xr[:, 13:] = 0.0
            ur[:, 4:] = 0.0

        # -------- sqp mode --------
        if is_sqp_change and t_sqp_start > t_sqp_end:
            if t_now >= t_sqp_start:
                ocp_solver.solver_options["nlp_solver_type"] = "SQP"

            if t_now >= t_sqp_end:
                ocp_solver.solver_options["nlp_solver_type"] = "SQP_RTI"

        # -------- update solver --------
        comp_time_start = time.time()

        if t_ctl >= ts_ctrl:
            t_ctl = 0.0

            # 0 ~ N-1
            for j in range(ocp_solver.N):
                yr = np.concatenate((xr[j, :], ur[j, :]))
                ocp_solver.set(j, "yref", yr)
                quaternion_r = xr[j, 6:10]
                params = np.zeros(n_param)
                params[0:4] = quaternion_r
                ocp_solver.set(j, "p", params)  # for nonlinear quaternion error

            # N
            yr = xr[ocp_solver.N, :]
            ocp_solver.set(ocp_solver.N, "yref", yr)  # final state of x, no u
            quaternion_r = xr[ocp_solver.N, 6:10]
            params = np.zeros(n_param)
            params[0:4] = quaternion_r
            ocp_solver.set(ocp_solver.N, "p", params)  # for nonlinear quaternion error

            # feedback, take the first action
            try:
                u_cmd = ocp_solver.solve_for_x0(x_now)
            except Exception as e:
                print(f"Round {i}: acados ocp_solver returned status {ocp_solver.status}. Exiting.")
                break

        comp_time_end = time.time()
        viz.comp_time[i] = comp_time_end - comp_time_start

        # -------- incremental nonlinear dynamic inverse --------
        # viz
        if hasattr(viz, "u_sim_mpc_all"):
            u_mpc = copy.deepcopy(u_cmd)
            viz.update_u_mpc(i, u_mpc)

        if args.dist_rej == 1:
            # wrench_meas
            sf_b, ang_acc_b = nmpc.fake_sensor.update_acc(x_now_sim)

            u_meas = np.zeros(8)
            u_meas[0:4] = x_now_sim[17:21]
            u_meas[4:] = x_now_sim[13:17]

            w = x_now_sim[10:13]
            mass = nmpc.fake_sensor.mass
            iv = nmpc.fake_sensor.iv

            wrench_meas = np.zeros(6)
            wrench_meas[0:3] = mass * sf_b
            wrench_meas[3:6] = np.dot(iv, ang_acc_b) + np.cross(w, np.dot(iv, w))

            # wrench_cmd
            ft_cmd = u_mpc[0:4]
            a_cmd = u_mpc[4:]

            z = np.zeros(8)
            z[0] = ft_cmd[0] * np.sin(a_cmd[0])
            z[1] = ft_cmd[0] * np.cos(a_cmd[0])
            z[2] = ft_cmd[1] * np.sin(a_cmd[1])
            z[3] = ft_cmd[1] * np.cos(a_cmd[1])
            z[4] = ft_cmd[2] * np.sin(a_cmd[2])
            z[5] = ft_cmd[2] * np.cos(a_cmd[2])
            z[6] = ft_cmd[3] * np.sin(a_cmd[3])
            z[7] = ft_cmd[3] * np.cos(a_cmd[3])

            wrench_cmd = np.dot(xr_ur_converter.alloc_mat, z)

            # B_inv
            wrench_cmd_tmp = np.dot(wrench_cmd.T, wrench_cmd)

            # make a matrix as B_inv = u_cmd @ wrench_cmd.T
            u_cmd_add_dim = np.expand_dims(u_cmd, axis=1)
            wrench_cmd_add_dim = np.expand_dims(wrench_cmd, axis=1)

            if wrench_cmd_tmp == 0:
                B_inv = np.dot(u_cmd_add_dim, (0 * wrench_cmd_add_dim.T))
            else:
                B_inv = np.dot(u_cmd_add_dim, (1 / wrench_cmd_tmp * wrench_cmd_add_dim.T))

            # indi
            d_u = np.dot(B_inv, (wrench_cmd - wrench_meas))

            u_cmd = u_meas + d_u

        # --------- update simulation ----------
        sim_solver.set("x", x_now_sim)
        sim_solver.set("u", u_cmd)

        status = sim_solver.solve()
        if status != 0:
            raise Exception(f"acados integrator returned status {status} in closed loop instance {i}")

        x_now_sim = sim_solver.get("x")

        # --------- update visualizer ----------
        viz.update(i, x_now_sim, u_cmd)  # note that the recording frequency of u_cmd is the same as ts_sim

    # ========== visualize ==========
    if args.plot_type == 0:
        viz.visualize(ocp_solver.acados_ocp.model.name, sim_solver.model_name, ts_ctrl, ts_sim, t_total_sim,
                      t_servo_ctrl=t_servo_ctrl, t_servo_sim=t_servo_sim)
    elif args.plot_type == 1:
        viz.visualize_less(ocp_solver.acados_ocp.model.name, sim_solver.model_name, ts_ctrl, ts_sim, t_total_sim,
                           t_servo_ctrl=t_servo_ctrl, t_servo_sim=t_servo_sim)
    elif args.plot_type == 2:
        viz.visualize_rpy(ocp_solver.acados_ocp.model.name, sim_solver.model_name, ts_ctrl, ts_sim, t_total_sim,
                          t_servo_ctrl=t_servo_ctrl, t_servo_sim=t_servo_sim)
