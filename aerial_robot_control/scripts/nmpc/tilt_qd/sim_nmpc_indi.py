"""
 Created by li-jinjie on 24-8-5.
"""
import copy
import time

import numpy as np
import argparse

from nmpc_viz import Visualizer

from tilt_qd_servo_thrust_dist import NMPCTiltQdServoThrustDist

np.random.seed(42)

if __name__ == "__main__":
    # read arguments
    parser = argparse.ArgumentParser(description="Run the simulation of different disturbance rejection methods.")
    parser.add_argument(
        "if_est_dist",
        type=int,
        help="Whether to estimate the disturbance. Options: 0 (no), 1 (yes)."
    )
    parser.add_argument(
        "indi_type",
        type=int,
        help="Whether to use INDI. Options: 0 (no), 1 (the B_inv is calculated using mpc command); "
             "2 (the B_inv is calculated using shifted mpc command); "
             "3 (the B_inv is calculated using sensor command); "
             "4 (the B_inv is calculated using the inverse of allocation matrix)."
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

    # --------- Disturb. Rej. ---------
    ts_sensor = 0.0025
    disturb_estimated = np.zeros(6)  # f_d_i, tau_d_b. Note that they are in different frames.
    disturb_nmpc_compd = np.zeros(6)  # f_d_i, tau_d_b. The disturbance that has been compensated by the nmpc.

    # ---------- Simulator ----------
    sim_nmpc = NMPCTiltQdServoThrustDist()
    t_servo_sim = getattr(sim_nmpc, "t_servo", 0.0)
    t_rotor_sim = getattr(sim_nmpc, "t_rotor", 0.0)

    ts_sim = 0.001

    disturb_init = np.zeros(6)
    disturb_init[0] = 1.0
    disturb_init[1] = -1.0
    disturb_init[2] = 1.0  # N, fz

    t_total_sim = 5.0
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
    x_init_sim[-6:] = disturb_init

    # ---------- Others ----------
    xr_ur_converter = nmpc.get_xr_ur_converter()
    viz = Visualizer(N_sim, nx_sim, nu, x_init_sim, is_record_diff_u=True)

    is_sqp_change = False
    t_sqp_start = 2.5
    t_sqp_end = 3.0

    # ========== update ==========
    u_cmd = u_init
    u_mpc = u_init
    t_ctl = 0.0
    t_sensor = 0.0
    x_now_sim = x_init_sim
    for i in range(N_sim):
        # --------- update time ---------
        t_now = i * ts_sim
        t_ctl += ts_sim
        t_sensor += ts_sim

        # --------- update state estimation ---------
        if isinstance(nmpc, NMPCTiltQdServoThrustDist):
            x_now = np.zeros(nx)
            x_now[:nx - 6] = x_now_sim[:nx - 6]  # copy elements except the last 6 elements, which are the disturbance
            x_now[-6:] = disturb_estimated
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
                u_mpc = ocp_solver.solve_for_x0(x_now)
                disturb_nmpc_compd = copy.deepcopy(disturb_estimated)
            except Exception as e:
                print(f"Round {i}: acados ocp_solver returned status {ocp_solver.status}. Exiting.")
                break

        comp_time_end = time.time()
        viz.comp_time[i] = comp_time_end - comp_time_start

        # -------- sensor-based disturbance rejection --------
        if hasattr(viz, "u_sim_mpc_all"):
            viz.update_u_mpc(i, u_mpc)

        # by default, the u_cmd is the mpc command
        if args.indi_type == 0:
            u_cmd = copy.deepcopy(u_mpc)

        # disturb est. is related to the sensor update frequency
        if t_sensor >= ts_sensor:
            t_sensor = 0.0

            # wrench_u_imu_b
            # - the wrench calculated from imu info
            sf_b, ang_acc_b, rot_ib = sim_nmpc.fake_sensor.update_acc(x_now_sim)

            w = x_now_sim[10:13]
            mass = sim_nmpc.fake_sensor.mass
            gravity = sim_nmpc.fake_sensor.gravity
            iv = sim_nmpc.fake_sensor.iv

            # add noise. real: scale = 0.00727 * gravity
            sf_b += np.random.normal(0.0, 0.1, 3)

            wrench_u_imu_b = np.zeros(6)
            wrench_u_imu_b[0:3] = mass * sf_b
            wrench_u_imu_b[3:6] = np.dot(iv, ang_acc_b) + np.cross(w, np.dot(iv, w))

            # wrench_u_sensor_b
            # - the wrench calculated from actuator sensor
            ft_sensor = x_now_sim[17:21]
            a_sensor = x_now_sim[13:17]

            z_sensor = np.zeros(8)
            z_sensor[0] = ft_sensor[0] * np.sin(a_sensor[0])
            z_sensor[1] = ft_sensor[0] * np.cos(a_sensor[0])
            z_sensor[2] = ft_sensor[1] * np.sin(a_sensor[1])
            z_sensor[3] = ft_sensor[1] * np.cos(a_sensor[1])
            z_sensor[4] = ft_sensor[2] * np.sin(a_sensor[2])
            z_sensor[5] = ft_sensor[2] * np.cos(a_sensor[2])
            z_sensor[6] = ft_sensor[3] * np.sin(a_sensor[3])
            z_sensor[7] = ft_sensor[3] * np.cos(a_sensor[3])

            wrench_u_sensor_b = np.dot(xr_ur_converter.alloc_mat, z_sensor)

            u_meas = np.zeros(8)
            u_meas[0:4] = ft_sensor
            u_meas[4:] = a_sensor

            # wrench_u_mpc_b
            # the wrench calculated from mpc command. the current command or the shifted command
            ft_mpc = u_mpc[0:4]
            a_mpc = u_mpc[4:]

            if args.indi_type == 2:
                ft_mpc = ft_sensor + (ts_ctrl / t_rotor_sim) * (u_mpc[0:4] - ft_sensor)
                a_mpc = a_sensor + (ts_ctrl / t_rotor_sim) * (u_mpc[4:] - a_sensor)

            z_mpc = np.zeros(8)
            z_mpc[0] = ft_mpc[0] * np.sin(a_mpc[0])
            z_mpc[1] = ft_mpc[0] * np.cos(a_mpc[0])
            z_mpc[2] = ft_mpc[1] * np.sin(a_mpc[1])
            z_mpc[3] = ft_mpc[1] * np.cos(a_mpc[1])
            z_mpc[4] = ft_mpc[2] * np.sin(a_mpc[2])
            z_mpc[5] = ft_mpc[2] * np.cos(a_mpc[2])
            z_mpc[6] = ft_mpc[3] * np.sin(a_mpc[3])
            z_mpc[7] = ft_mpc[3] * np.cos(a_mpc[3])

            wrench_u_mpc_b = np.dot(xr_ur_converter.alloc_mat, z_mpc)

            # update disturbance estimation
            if args.if_est_dist == 1:
                # only use the wrench difference between the imu and the actuator sensor, no u_mpc
                alpha = 0.005
                disturb_estimated[0:3] = (1 - alpha) * disturb_estimated[0:3] + alpha * np.dot(rot_ib, (
                        wrench_u_imu_b[0:3] - wrench_u_sensor_b[0:3]))  # world frame
                disturb_estimated[3:6] = (1 - alpha) * disturb_estimated[3:6] + alpha * (
                        wrench_u_imu_b[3:6] - wrench_u_sensor_b[3:6])  # body frame

            # --- for the methods that needs to update u_cmd, such as INDI ---
            if args.indi_type > 0:
                # the disturbance estimation residual
                dist_wrench_res_b = np.zeros(6)
                dist_est_now_b = wrench_u_imu_b - wrench_u_sensor_b
                dist_wrench_res_b[0:3] = dist_est_now_b[0:3] - np.dot(rot_ib.T, disturb_nmpc_compd[0:3])
                dist_wrench_res_b[3:6] = dist_est_now_b[3:6] - disturb_nmpc_compd[3:6]

            if args.indi_type > 0 and args.indi_type != 4:
                # B_inv
                wrench_u_b = wrench_u_sensor_b if args.indi_type == 3 else wrench_u_mpc_b
                wrench_2norm_sq = np.dot(wrench_u_b.T, wrench_u_b)
                if wrench_2norm_sq == 0:
                    B_inv = np.dot(np.expand_dims(u_mpc, axis=1), (0 * np.expand_dims(wrench_u_b, axis=1).T))
                else:
                    B_inv = np.dot(np.expand_dims(u_mpc, axis=1),
                                   (1 / wrench_2norm_sq * np.expand_dims(wrench_u_b, axis=1).T))

                # NOTE THAT d_u should be negatively related to dist_wrench_res_b
                d_u = np.dot(B_inv, -dist_wrench_res_b)

                u_cmd = copy.deepcopy(u_mpc + d_u)

            if args.indi_type == 4:
                d_z = np.dot(xr_ur_converter.alloc_mat_pinv, -dist_wrench_res_b)
                z = z_mpc + d_z

                u_cmd = np.zeros(8)
                u_cmd[0] = np.sqrt(z[0] ** 2 + z[1] ** 2)  # ft
                u_cmd[1] = np.sqrt(z[2] ** 2 + z[3] ** 2)
                u_cmd[2] = np.sqrt(z[4] ** 2 + z[5] ** 2)
                u_cmd[3] = np.sqrt(z[6] ** 2 + z[7] ** 2)
                u_cmd[4] = np.arctan2(z[0], z[1])  # alpha
                u_cmd[5] = np.arctan2(z[2], z[3])
                u_cmd[6] = np.arctan2(z[4], z[5])
                u_cmd[7] = np.arctan2(z[6], z[7])

        # --------- update simulation ----------
        disturb = copy.deepcopy(disturb_init)
        # disturb[2] = np.random.normal(1.0, 3.0)  # N, fz
        if 2.0 <= t_now < 3.0:
            disturb[2] = -5.0

        x_now_sim[-6:] = disturb

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
