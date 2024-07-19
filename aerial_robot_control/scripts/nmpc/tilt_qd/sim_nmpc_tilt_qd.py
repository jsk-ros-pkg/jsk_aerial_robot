"""
 Created by li-jinjie on 24-3-9.
"""
import time

import numpy as np
import argparse
from acados_template import AcadosModel, AcadosSim, AcadosSimSolver

from nmpc_viz import Visualizer

# naive models
from tilt_qd_no_servo import NMPCTiltQdNoServo
from tilt_qd_no_servo_new_cost import NMPCTiltQdNoServoNewCost

# consider the servo delay
from tilt_qd_servo import NMPCTiltQdServo
from tilt_qd_servo_dist import NMPCTiltQdServoDist
from tilt_qd_servo_drag_w_dist import NMPCTiltQdServoDragDist

from tilt_qd_servo_old_cost import NMPCTiltQdServoOldCost
from tilt_qd_servo_vel_input import NMPCTiltQdServoVelInput

# further consider the thrust delay
from tilt_qd_servo_thrust import NMPCTiltQdServoThrust
from tilt_qd_servo_thrust_drag import NMPCTiltQdServoThrustDrag

if __name__ == "__main__":
    # read arguments
    parser = argparse.ArgumentParser(description="Run the simulation of different NMPC models.")
    parser.add_argument(
        "model",
        type=int,
        help="The NMPC model to be simulated. Options: 0 (no_servo_delay), 1 (old servo cost), 2 (full).",
    )
    parser.add_argument("-sim", "--sim_model", type=int, default=0,
                        help="The simulation model. "
                             "Options: 0 (default: NMPCTiltQdServoThrust), 1 (NMPCTiltQdServoThrustDrag).")
    parser.add_argument("-p", "--plot_type", type=int, default=0, help="The type of plot. Options: 0 (full), 1, 2.")

    args = parser.parse_args()

    # ========== init ==========
    # ---------- Controller ----------
    if args.model == 0:
        nmpc = NMPCTiltQdNoServo()
    elif args.model == 1:
        nmpc = NMPCTiltQdServoOldCost()
    elif args.model == 2:
        nmpc = NMPCTiltQdNoServoNewCost()
    elif args.model == 3:
        nmpc = NMPCTiltQdServo()
    elif args.model == 4:
        nmpc = NMPCTiltQdServoDist()
    elif args.model == 5:
        nmpc = NMPCTiltQdServoVelInput()
        alpha_integ = np.zeros(4)
    elif args.model == 6:
        nmpc = NMPCTiltQdServoThrust()
    elif args.model == 7:
        nmpc = NMPCTiltQdServoDragDist()
    else:
        raise ValueError(f"Invalid control model {args.model}.")

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
    if args.sim_model == 0:
        sim_nmpc = NMPCTiltQdServoThrust()  # consider both the servo delay and the control delay
    elif args.sim_model == 1:
        sim_nmpc = NMPCTiltQdServoThrustDrag()
    else:
        raise ValueError(f"Invalid sim model {args.sim_model}.")

    if hasattr(sim_nmpc, "t_servo"):
        t_servo_sim = sim_nmpc.t_servo
    else:
        t_servo_sim = 0.0

    ts_sim = 0.005

    t_total_sim = 15.0
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
        if isinstance(nmpc, NMPCTiltQdServoDist):
            x_now = np.zeros(nx)
            x_now[:nx - 6] = x_now_sim[:nx - 6]  # only remove the last 6 elements, which are the disturbances
        else:
            x_now = x_now_sim[:nx]  # the dimension of x_now may be smaller than x_now_sim

        # -------- update control target --------
        target_xyz = np.array([[0.3, 0.6, 1.0]]).T
        target_rpy = np.array([[0.0, 0.0, 0.0]]).T

        if args.plot_type == 2:
            target_xyz = np.array([[0.0, 0.0, 0.0]]).T
            target_rpy = np.array([[0.5, 0.5, 0.5]]).T

        if t_total_sim > 2.0:
            if 2.0 <= t_now < 6:
                target_xyz = np.array([[0.3, 0.6, 1.0]]).T

                roll = 30.0 / 180.0 * np.pi
                pitch = 60.0 / 180.0 * np.pi
                yaw = 90.0 / 180.0 * np.pi
                target_rpy = np.array([[roll, pitch, yaw]]).T

            if t_now >= 6:
                assert t_sqp_end <= 3.0
                target_xyz = np.array([[1.0, 1.0, 1.0]]).T
                target_rpy = np.array([[0.0, 0.0, 0.0]]).T

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

        # if nmpc is NMPCOverActNoServoNewCost
        if type(nmpc) is NMPCTiltQdNoServoNewCost:
            xr_ur_converter.update_a_prev(u_cmd.item(4), u_cmd.item(5), u_cmd.item(6), u_cmd.item(7))

        if type(nmpc) is NMPCTiltQdServoVelInput:
            alpha_integ += u_cmd[4:] * ts_ctrl
            u_cmd[4:] = alpha_integ  # convert from delta to absolute

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
