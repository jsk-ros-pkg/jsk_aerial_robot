"""
 Created by li-jinjie on 24-3-9.
"""
import copy
import time
import numpy as np
import argparse
import os

from nmpc_viz import Visualizer

# naive models
from tilt_qd_no_servo import NMPCTiltQdNoServo
from tilt_qd_no_servo_new_cost import NMPCTiltQdNoServoNewCost

# consider the servo delay
from tilt_qd_servo import NMPCTiltQdServo
from tilt_qd_servo_dist import NMPCTiltQdServoDist
from arxiv_tilt_qd_servo_drag_w_dist import NMPCTiltQdServoDragDist

from tilt_qd_servo_old_cost import NMPCTiltQdServoOldCost
from tilt_qd_servo_vel_input import NMPCTiltQdServoVelInput

# further consider the thrust delay
from tilt_qd_servo_thrust import NMPCTiltQdServoThrust
from tilt_qd_servo_thrust_dist import NMPCTiltQdServoThrustDist
from arxiv_tilt_qd_servo_thrust_drag import NMPCTiltQdServoThrustDrag

# only consider the thrust delay
from tilt_qd_thrust import NMPCTiltQdThrust


def simulate(nmpc_model_id, sim_model_id=0, plot_type=1, no_viz=True, save_data=False, file_path=None):
    # ========== init ==========
    # ---------- Controller ----------
    if nmpc_model_id == 0:
        nmpc = NMPCTiltQdNoServo()
    elif nmpc_model_id == 1:
        nmpc = NMPCTiltQdServo()
    elif nmpc_model_id == 2:
        nmpc = NMPCTiltQdThrust()
    elif nmpc_model_id == 3:
        nmpc = NMPCTiltQdServoThrust()
    elif nmpc_model_id == 21:
        nmpc = NMPCTiltQdServoDist()
    elif nmpc_model_id == 22:
        nmpc = NMPCTiltQdServoThrustDist()

    # archiving methods
    elif nmpc_model_id == 91:
        nmpc = NMPCTiltQdNoServoNewCost()
    elif nmpc_model_id == 92:
        nmpc = NMPCTiltQdServoOldCost()
    elif nmpc_model_id == 93:
        nmpc = NMPCTiltQdServoVelInput()
        alpha_integ = np.zeros(4)
    elif nmpc_model_id == 94:
        nmpc = NMPCTiltQdServoDragDist()
    else:
        raise ValueError(f"Invalid control model {nmpc_model_id}.")

    if hasattr(nmpc, "t_servo"):
        t_servo_ctrl = nmpc.t_servo
    else:
        t_servo_ctrl = 0.0
    ts_ctrl = nmpc.ts_ctrl

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
    if sim_model_id == 0:
        sim_nmpc = NMPCTiltQdServoThrust()  # consider the dynamics of all actuators, including servo and thrust
    elif sim_model_id == 1:
        sim_nmpc = NMPCTiltQdServoThrustDrag()
    else:
        raise ValueError(f"Invalid sim model {sim_model_id}.")

    if hasattr(sim_nmpc, "t_servo"):
        t_servo_sim = sim_nmpc.t_servo
    else:
        t_servo_sim = 0.0

    ts_sim = 0.005

    t_total_sim = 15.0
    if plot_type == 1:
        t_total_sim = 4.0
    if plot_type == 2:
        t_total_sim = 3.0

    N_sim = int(t_total_sim / ts_sim)

    # sim solver
    sim_solver = sim_nmpc.create_acados_sim_solver(sim_nmpc.get_ocp_model(), ts_sim, True)
    nx_sim = sim_solver.acados_sim.dims.nx

    x_init_sim = np.zeros(nx_sim)
    x_init_sim[6] = 1.0  # qw

    # ---------- Others ----------
    xr_ur_converter = nmpc.get_xr_ur_converter()

    # Create visualizer only if visualization is not disabled
    if not no_viz:
        viz = Visualizer(N_sim, nx_sim, nu, x_init_sim)

    # Prepare containers to record simulation data (x and u) for future comparison
    x_history = []
    u_history = []

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
        if isinstance(nmpc, NMPCTiltQdServoDist) or isinstance(nmpc, NMPCTiltQdServoThrustDist):
            x_now = np.zeros(nx)
            x_now[: nx - 6] = copy.deepcopy(x_now_sim[: nx - 6]) # only remove the last 6 elements (disturbances)
        else:
            x_now = copy.deepcopy(x_now_sim[:nx])  # the dimension of x_now may be smaller than x_now_sim

        if isinstance(nmpc, NMPCTiltQdThrust):
            x_now[13:17] = copy.deepcopy(x_now_sim[17:21])

        # -------- update control target --------
        target_xyz = np.array([[0.3, 0.6, 1.0]]).T
        target_rpy = np.array([[0.0, 0.0, 0.0]]).T

        if plot_type == 2:
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

        if plot_type == 2:
            if nx > 13:
                xr[:, 13:] = 0.0
            ur[:, 4:] = 0.0

        # -------- SQP mode --------
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
        if not no_viz:
            viz.comp_time[i] = comp_time_end - comp_time_start

        if type(nmpc) is NMPCTiltQdNoServoNewCost:
            xr_ur_converter.update_a_prev(u_cmd.item(4), u_cmd.item(5), u_cmd.item(6), u_cmd.item(7))
        if type(nmpc) is NMPCTiltQdServoVelInput:
            alpha_integ += u_cmd[4:] * ts_ctrl
            u_cmd[4:] = alpha_integ  # convert from delta input to real input

        # --------- update simulation ----------
        sim_solver.set("x", x_now_sim)
        sim_solver.set("u", u_cmd)

        status = sim_solver.solve()
        if status != 0:
            raise Exception(f"acados integrator returned status {status} in closed loop instance {i}")

        x_now_sim = sim_solver.get("x")

        # Save current simulation data for later comparison
        x_history.append(x_now_sim.copy())
        u_history.append(u_cmd.copy())

        # Update visualization only if enabled
        if not no_viz:
            viz.update(i, x_now_sim, u_cmd)

    # ========== visualize ==========
    if not no_viz:
        if plot_type == 0:
            viz.visualize(
                ocp_solver.acados_ocp.model.name,
                sim_solver.model_name,
                ts_ctrl,
                ts_sim,
                t_total_sim,
                t_servo_ctrl=t_servo_ctrl,
                t_servo_sim=t_servo_sim,
            )
        elif plot_type == 1:
            viz.visualize_less(
                ocp_solver.acados_ocp.model.name,
                sim_solver.model_name,
                ts_ctrl,
                ts_sim,
                t_total_sim,
                t_servo_ctrl=t_servo_ctrl,
                t_servo_sim=t_servo_sim,
            )
        elif plot_type == 2:
            viz.visualize_rpy(
                ocp_solver.acados_ocp.model.name,
                sim_solver.model_name,
                ts_ctrl,
                ts_sim,
                t_total_sim,
                t_servo_ctrl=t_servo_ctrl,
                t_servo_sim=t_servo_sim,
            )
    else:
        print("Visualization has been disabled by the user.")

    # Save the simulation data if requested
    if save_data:
        # os.makedirs(file_path, exist_ok=True)
        data_file = os.path.join(file_path, f"nmpc_{type(nmpc).__name__}_model_{type(sim_nmpc).__name__}.npz")
        np.savez(data_file, x=np.array(x_history), u=np.array(u_history))
        print(f"Simulation data saved to {data_file}")

    return np.array(x_history), np.array(u_history)


if __name__ == "__main__":
    # read arguments
    parser = argparse.ArgumentParser(description="Run the simulation of different NMPC models.")
    parser.add_argument(
        "model",
        type=int,
        help="The NMPC model to be simulated. Options: 0 (no_servo_delay), 1 (old servo cost), 2 (full).",
    )
    parser.add_argument(
        "-sim",
        "--sim_model",
        type=int,
        default=0,
        help="The simulation model. Options: 0 (default: NMPCTiltQdServoThrust), 1 (NMPCTiltQdServoThrustDrag).",
    )
    parser.add_argument("-p", "--plot_type", type=int, default=0, help="The type of plot. Options: 0 (full), 1, 2.")

    # New option 1: disable visualization
    parser.add_argument("--no_viz", action="store_true", help="Disable visualization after simulation")
    # New option 2: save simulation data (x and u) to file
    parser.add_argument("--save_data", action="store_true", help="Save simulation x and u data to file")
    parser.add_argument("--file_path", type=str, default="../../../../test/tilt_qd/", help="Path to save the data file")

    args = parser.parse_args()

    simulate(args.model, args.sim_model, args.plot_type, args.no_viz, args.save_data, args.file_path)
