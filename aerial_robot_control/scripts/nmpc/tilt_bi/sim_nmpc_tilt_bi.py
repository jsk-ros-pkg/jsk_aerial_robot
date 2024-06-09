"""
 Created by li-jinjie on 24-3-9.
"""
import time

import numpy as np
import argparse
from acados_template import AcadosModel, AcadosSim, AcadosSimSolver

from nmpc_viz import Visualizer

from tilt_bi_full import NMPCTiltBiFull
from tilt_bi_2ord_servo import NMPCTiltBi2OrdServo


def create_acados_sim_solver(ocp_model: AcadosModel, ts_sim: float) -> AcadosSimSolver:
    acados_sim = AcadosSim()
    acados_sim.model = ocp_model
    n_params = ocp_model.p.size()[0]
    acados_sim.dims.np = n_params  # TODO: seems that the np needn't to be set manually in the latest version of acados
    acados_sim.parameter_values = np.zeros(n_params)

    acados_sim.solver_options.T = ts_sim
    # important to sim 2-ord servo model
    acados_sim.solver_options.integrator_type = 'IRK'
    acados_sim.solver_options.num_stages = 3
    acados_sim.solver_options.num_steps = 3
    acados_sim.solver_options.newton_iter = 3  # for implicit integrator
    acados_sim.solver_options.collocation_type = "GAUSS_RADAU_IIA"

    acados_sim_solver = AcadosSimSolver(acados_sim, json_file="acados_ocp_" + ocp_model.name + ".json")
    return acados_sim_solver


if __name__ == "__main__":
    # read arguments
    parser = argparse.ArgumentParser(description="Run the simulation of different NMPC models.")
    parser.add_argument(
        "model",
        type=int,
        help="The NMPC model to be simulated. Options: 0 (full), 1 (full w. integrator).",
    )
    parser.add_argument("-p", "--plot_type", type=int, default=0, help="The type of plot. Options: 0 (full), 1, 2.")

    args = parser.parse_args()

    # ========== init ==========
    # ---------- Controller ----------
    if args.model == 0:
        nmpc = NMPCTiltBiFull()
    elif args.model == 1:
        nmpc = NMPCTiltBi2OrdServo()
    else:
        raise ValueError(f"Invalid model {args.model}.")

    # check if there is t_servo in the controller
    if hasattr(nmpc, "t_servo"):
        t_servo_ctrl = nmpc.t_servo
    else:
        t_servo_ctrl = 0.0
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
    sim_nmpc = NMPCTiltBi2OrdServo()

    if hasattr(sim_nmpc, "t_servo"):
        t_servo_sim = sim_nmpc.t_servo
    else:
        t_servo_sim = 0.0

    ts_sim = 0.001

    t_total_sim = 15.0
    if args.plot_type == 1:
        t_total_sim = 2.0
    if args.plot_type == 2:
        t_total_sim = 3.0

    N_sim = int(t_total_sim / ts_sim)

    # sim solver
    sim_nmpc.get_ocp_model()
    sim_solver = create_acados_sim_solver(sim_nmpc.get_ocp_model(), ts_sim)
    nx_sim = sim_solver.acados_sim.dims.nx

    x_init_sim = np.zeros(nx_sim)
    x_init_sim[6] = 1.0  # qw

    # ---------- Others ----------
    xr_ur_converter = nmpc.get_xr_ur_converter()
    viz = Visualizer(N_sim, nx_sim, nu, x_init_sim)

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
        x_now = x_now_sim[:nx]  # the dimension of x_now may be smaller than x_now_sim

        # -------- update control target --------
        target_xyz = np.array([[0.0, 0.0, 1.0]]).T
        target_rpy = np.array([[0.0, 0.0, 0.0]]).T

        if args.plot_type == 2:
            target_xyz = np.array([[0.0, 0.0, 0.0]]).T
            target_rpy = np.array([[0.5, 0.5, 0.5]]).T

        if t_total_sim > 3.0:
            if 3.0 <= t_now < 5.5:
                assert t_sqp_end <= 3.0
                target_xyz = np.array([[1.0, 0.0, 1.0]]).T
                target_rpy = np.array([[0.0, 0.0, 0.0]]).T

            # if t_now >= 5.5:
            #     target_xyz = np.array([[1.0, 1.0, 1.0]]).T
            #
            #     roll = 30.0 / 180.0 * np.pi
            #     pitch = 0.0 / 180.0 * np.pi
            #     yaw = 0.0 / 180.0 * np.pi
            #     target_rpy = np.array([[roll, pitch, yaw]]).T

        xr, ur = xr_ur_converter.pose_point_2_xr_ur(target_xyz, target_rpy)

        if args.plot_type == 2:
            if nx > 13:
                xr[:, 13:] = 0.0
            ur[:, 2:] = 0.0

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
        viz.visualize(
            ocp_solver.acados_ocp.model.name,
            sim_solver.model_name,
            ts_ctrl,
            ts_sim,
            t_total_sim,
            t_servo_ctrl=t_servo_ctrl,
            t_servo_sim=t_servo_sim,
        )
    elif args.plot_type == 1:
        viz.visualize_less(
            ocp_solver.acados_ocp.model.name,
            sim_solver.model_name,
            ts_ctrl,
            ts_sim,
            t_total_sim,
            t_servo_ctrl=t_servo_ctrl,
            t_servo_sim=t_servo_sim,
        )
    elif args.plot_type == 2:
        viz.visualize_rpy(
            ocp_solver.acados_ocp.model.name,
            sim_solver.model_name,
            ts_ctrl,
            ts_sim,
            t_total_sim,
            t_servo_ctrl=t_servo_ctrl,
            t_servo_sim=t_servo_sim,
        )
