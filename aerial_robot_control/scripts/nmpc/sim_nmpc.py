import sys, os
from copy import deepcopy
import time
import numpy as np
import argparse

try:
    # For relative import in module
    from .nmpc_viz import Visualizer
    
    # Quadrotor
    from .tilt_qd import phys_param_beetle_omni as phys_omni
    from .archive import phys_param_beetle_art as phys_art

    # - Naive models
    from .tilt_qd.tilt_qd_no_servo import NMPCTiltQdNoServo
    from .archive.tilt_qd_no_servo_ac_cost import NMPCTiltQdNoServoAcCost

    # - Consider the servo delay with its model
    from .tilt_qd.tilt_qd_servo import NMPCTiltQdServo
    from .tilt_qd.tilt_qd_servo_dist import NMPCTiltQdServoDist
    from .archive.tilt_qd_servo_drag_w_dist import NMPCTiltQdServoDragDist
    from .archive.tilt_qd_servo_w_cog_end_dist import NMPCTiltQdServoWCogEndDist

    # - Conside servo angle derivative as state
    from .tilt_qd.tilt_qd_servo_diff import NMPCTiltQdServoDiff

    # - Consider the absolute servo angle command in cost
    from .archive.tilt_qd_servo_old_cost import NMPCTiltQdServoOldCost

    # - Consider the thrust delay with its model
    from .tilt_qd.tilt_qd_thrust import NMPCTiltQdThrust

    # - Consider the servo & thrust delay with its models
    from .tilt_qd.tilt_qd_servo_thrust import NMPCTiltQdServoThrust
    from .tilt_qd.tilt_qd_servo_thrust_dist import NMPCTiltQdServoThrustDist
    from .archive.tilt_qd_servo_thrust_drag import NMPCTiltQdServoThrustDrag

    # Birotor
    from .tilt_bi.tilt_bi_servo import NMPCTiltBiServo
    from .tilt_bi.tilt_bi_2ord_servo import NMPCTiltBi2OrdServo

    # Trirotor
    from .tilt_tri.tilt_tri_servo import NMPCTiltTriServo
    from .tilt_tri.tilt_tri_servo_dist import NMPCTiltTriServoDist
    
except ImportError:
    # For relative import in script
    sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/tilt_bi")
    sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/tilt_tri")
    sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/tilt_qd")
    sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/archive")

    from nmpc_viz import Visualizer

    # Quadrotor
    import tilt_qd.phys_param_beetle_omni as phys_omni
    import archive.phys_param_beetle_art as phys_art

    # - Naive models
    from archive.tilt_qd_no_servo_ac_cost import NMPCTiltQdNoServoAcCost
    from tilt_qd.tilt_qd_no_servo import NMPCTiltQdNoServo

    # - Consider the servo delay with its model
    from tilt_qd.tilt_qd_servo import NMPCTiltQdServo
    from tilt_qd.tilt_qd_servo_dist import NMPCTiltQdServoDist
    from archive.tilt_qd_servo_drag_w_dist import NMPCTiltQdServoDragDist
    from archive.tilt_qd_servo_w_cog_end_dist import NMPCTiltQdServoWCogEndDist

    # - Conside servo angle derivative as state
    from tilt_qd.tilt_qd_servo_diff import NMPCTiltQdServoDiff
    
    # - Consider the absolute servo angle command in cost
    from archive.tilt_qd_servo_old_cost import NMPCTiltQdServoOldCost

    # - Consider the thrust delay with its model
    from tilt_qd.tilt_qd_thrust import NMPCTiltQdThrust

    # - Consider the servo & thrust delay with its models
    from tilt_qd.tilt_qd_servo_thrust import NMPCTiltQdServoThrust
    from tilt_qd.tilt_qd_servo_thrust_dist import NMPCTiltQdServoThrustDist
    from archive.tilt_qd_servo_thrust_drag import NMPCTiltQdServoThrustDrag

    # Birotor
    from tilt_bi.tilt_bi_servo import NMPCTiltBiServo
    from tilt_bi.tilt_bi_2ord_servo import NMPCTiltBi2OrdServo

    # Trirotor
    from tilt_tri.tilt_tri_servo import NMPCTiltTriServo
    from tilt_tri.tilt_tri_servo_dist import NMPCTiltTriServoDist


def main(args):
    # ========== Init ==========
    # ---------- Controller ----------
    if args.arch == 'qd':

        if args.model == 0:
            nmpc = NMPCTiltQdNoServo(phys=phys_art)
        elif args.model == 1:
            nmpc = NMPCTiltQdServo(phys=phys_art)
        elif args.model == 2:
            nmpc = NMPCTiltQdThrust(phys=phys_art)
        elif args.model == 3:
            nmpc = NMPCTiltQdServoThrust(phys=phys_art)

        elif args.model == 21:
            nmpc = NMPCTiltQdServoDist(phys=phys_omni)
        elif args.model == 22:
            nmpc = NMPCTiltQdServoThrustDist(phys=phys_omni)

        # Archived methods
        elif args.model == 91:
            nmpc = NMPCTiltQdNoServoAcCost()
        elif args.model == 92:
            nmpc = NMPCTiltQdServoOldCost()
        elif args.model == 93:
            nmpc = NMPCTiltQdServoDiff()
            alpha_integ = np.zeros(4)
        elif args.model == 94:
            nmpc = NMPCTiltQdServoDragDist()
        elif args.model == 95:
            nmpc = NMPCTiltQdServoThrustDrag()
        elif args.model == 96:
            nmpc = NMPCTiltQdServoWCogEndDist()
        else:
            raise ValueError(f"Invalid control model {args.model}.")

    elif args.arch == 'bi':

        if args.model == 0:
            nmpc = NMPCTiltBiServo()
        elif args.model == 1:
            nmpc = NMPCTiltBi2OrdServo()
        else:
            raise ValueError(f"Invalid model {args.model}.")

    elif args.arch == 'tri':

        if args.model == 0:
            nmpc = NMPCTiltTriServo()
        elif args.model == 1:
            nmpc = NMPCTiltTriServoDist()
        else:
            raise ValueError(f"Invalid model {args.model}.")

    else:
        raise ValueError(f"Invalid robot architecture {args.arch}.")

    # Get time constants
    if nmpc.include_servo_model:
        t_servo_ctrl = nmpc.phys.t_servo
    else:
        t_servo_ctrl = 0.0
    ts_ctrl = nmpc.params["T_samp"]

    # OCP solver
    ocp_solver = nmpc.get_ocp_solver()
    nx = ocp_solver.acados_ocp.dims.nx
    nu = ocp_solver.acados_ocp.dims.nu
    n_param = ocp_solver.acados_ocp.dims.np

    x_init = np.zeros(nx)
    x_init[6] = 1.0  # qw
    u_init = np.zeros(nu)

    for stage in range(ocp_solver.N + 1):
        ocp_solver.set(stage, "x", x_init)
    for stage in range(ocp_solver.N):
        ocp_solver.set(stage, "u", u_init)

    # ---------- Simulator ----------
    if args.arch == 'qd':

        sim_phy = phys_omni if 20 < args.model < 30 else phys_art
        if args.sim_model == 0:
            sim_nmpc = NMPCTiltQdServoThrust(phys=sim_phy)  # Consider both the servo delay and the thrust delay
        elif args.sim_model == 1:
            sim_nmpc = NMPCTiltQdServoThrustDrag(phys=sim_phy)  # Also consider drag in wrench formulation
        else:
            raise ValueError(f"Invalid sim model {args.sim_model}.")

    elif args.arch == 'bi':

        if args.sim_model == 0:
            sim_nmpc = NMPCTiltBiServo()
        # elif args.sim_model == 1:
        #     sim_nmpc = NMPCTiltBi2OrdServo()   # This model is wrong
        else:
            raise ValueError(f"Invalid sim model {args.sim_model}.")

    elif args.arch == 'tri':

        sim_nmpc = NMPCTiltTriServo()

    else:
        raise ValueError(f"Invalid robot architecture {args.arch}.")

    # Get time constants
    if sim_nmpc.include_servo_model:
        t_servo_sim = sim_nmpc.phys.t_servo
    else:
        t_servo_sim = 0.0
    if sim_nmpc.include_thrust_model:
        t_rotor_sim = sim_nmpc.phys.t_rotor
    else:
        t_rotor_sim = 0.0

    ts_sim = 0.005  # or 0.001

    t_total_sim = 15.0
    if args.plot_type == 1:
        t_total_sim = 4.0
    if args.plot_type == 2:
        t_total_sim = 3.0

    N_sim = int(t_total_sim / ts_sim)

    # Sim solver
    sim_solver = sim_nmpc.create_acados_sim_solver(ts_sim, is_build=True)
    nx_sim = sim_solver.acados_sim.dims.nx

    # State Initialization
    x_init_sim = np.zeros(nx_sim)
    x_init_sim[6] = 1.0  # qw

    # ---------- Reference ----------
    reference_generator = nmpc.get_reference_generator()

    # ---------- Visualization ----------
    viz = Visualizer(
        args.arch,
        N_sim,
        nx_sim,
        nu,
        x_init_sim,
        tilt=nmpc.tilt,
        include_servo_model=sim_nmpc.include_servo_model,
        include_thrust_model=sim_nmpc.include_thrust_model,
        include_cog_dist_model=sim_nmpc.include_cog_dist_model
    )

    # Prepare containers to record simulation data (x and u) for future comparison
    x_history = []
    u_history = []

    is_sqp_change = False
    t_sqp_start = 2.5
    t_sqp_end = 3.0

    # ========== Run simulation ==========
    u_cmd = u_init
    t_ctl = 0.0
    x_now_sim = x_init_sim
    for i in range(N_sim):
        # --------- Update time ---------
        t_now = i * ts_sim
        t_ctl += ts_sim

        # --------- Update state estimation ---------
        # Assemble state from simulation and disturbance estimation 
        if nmpc.include_cog_dist_model:
            x_now = np.zeros(nx)
            x_now[: nx - 6] = deepcopy(x_now_sim[: nx - 6])
        else:
            x_now = deepcopy(x_now_sim[:nx])  # The dimension of x_now may be smaller than x_now_sim

        # Access from less indices
        if (nmpc.include_thrust_model and not nmpc.include_servo_model) and (
                sim_nmpc.include_servo_model and sim_nmpc.include_thrust_model):
            if args.arch == 'bi':
                x_now[13:15] = deepcopy(x_now_sim[15:17])
            elif args.arch == 'tri':
                x_now[13:16] = deepcopy(x_now_sim[16:19])
            elif args.arch == 'qd':
                x_now[13:17] = deepcopy(x_now_sim[17:21])

        # -------- Update control target --------
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

            # if 3.0 <= t_now < 5.5:
            #     assert t_sqp_end <= 3.0
            #     target_xyz = np.array([[1.0, 1.0, 1.0]]).T
            #     target_rpy = np.array([[0.0, 0.0, 0.0]]).T
            # if t_now >= 5.5:
            #     target_xyz = np.array([[1.0, 1.0, 1.0]]).T

            #     roll = 30.0 / 180.0 * np.pi
            #     pitch = 0.0 / 180.0 * np.pi
            #     yaw = 0.0 / 180.0 * np.pi
            #     target_rpy = np.array([[roll, pitch, yaw]]).T

            if t_now >= 6:
                assert t_sqp_end <= 3.0
                target_xyz = np.array([[1.0, 1.0, 1.0]]).T
                target_rpy = np.array([[0.0, 0.0, 0.0]]).T

        # Compute reference trajectory from target pose
        xr, ur = reference_generator.compute_trajectory(target_xyz, target_rpy)

        if args.plot_type == 2:
            if nx > 13:
                xr[:, 13:] = 0.0
            if args.arch == 'bi':
                ur[:, 2:] = 0.0
            elif args.arch == 'tri':
                ur[:, 3:] = 0.0
            elif args.arch == 'qd':
                ur[:, 4:] = 0.0

        # -------- Set SQP mode --------
        if is_sqp_change and t_sqp_start > t_sqp_end:
            if t_now >= t_sqp_start:
                ocp_solver.solver_options["nlp_solver_type"] = "SQP"

            if t_now >= t_sqp_end:
                ocp_solver.solver_options["nlp_solver_type"] = "SQP_RTI"

        # -------- Update solver --------
        comp_time_start = time.time()

        if t_ctl >= ts_ctrl:
            t_ctl = 0.0

            # 0 ~ N-1
            for j in range(ocp_solver.N):
                yr = np.concatenate((xr[j, :], ur[j, :]))
                ocp_solver.set(j, "yref", yr)
                quaternion_r = xr[j, 6:10]
                nmpc.acados_init_p[0:4] = quaternion_r
                ocp_solver.set(j, "p", nmpc.acados_init_p)  # For nonlinear quaternion error

            # N
            yr = xr[ocp_solver.N, :]
            ocp_solver.set(ocp_solver.N, "yref", yr)  # Final state of x, no u
            quaternion_r = xr[ocp_solver.N, 6:10]
            nmpc.acados_init_p[0:4] = quaternion_r
            ocp_solver.set(ocp_solver.N, "p", nmpc.acados_init_p)  # For nonlinear quaternion error

            # Compute control feedback and take the first action
            try:
                u_cmd = ocp_solver.solve_for_x0(x_now)
            except Exception as e:
                print(f"Round {i}: acados ocp_solver returned status {ocp_solver.status}. Exiting.")
                break

        comp_time_end = time.time()
        viz.comp_time[i] = comp_time_end - comp_time_start

        if args.arch == 'qd':
            # Use previous servo angle as reference
            if type(nmpc) is NMPCTiltQdNoServoAcCost:
                nmpc.update_a_prev(u_cmd.item(4), u_cmd.item(5), u_cmd.item(6), u_cmd.item(7))

            # Use servo angle derivative as state and therefore integrate servo angle command
            if nmpc.include_servo_derivative:
                alpha_integ += u_cmd[4:] * ts_ctrl
                u_cmd[4:] = alpha_integ  # convert from delta input to real input

        # --------- Update simulation ----------
        sim_solver.set("x", x_now_sim)
        sim_solver.set("u", u_cmd)

        status = sim_solver.solve()
        if status != 0:
            raise Exception(f"acados integrator returned status {status} in closed loop instance {i}")

        x_now_sim = sim_solver.get("x")

        # Save current simulation data for later comparison
        x_history.append(x_now_sim.copy())
        u_history.append(u_cmd.copy())

        # --------- Update visualizer ----------
        viz.update(i, x_now_sim, u_cmd)  # Note: The recording frequency of u_cmd is the same as ts_sim

    # ========== Visualize ==========
    if not args.no_viz:
        if args.plot_type == 0:
            viz.visualize(
                ocp_solver.acados_ocp.model.name,
                sim_solver.model_name,
                ts_ctrl,
                ts_sim,
                t_total_sim,
                t_servo_ctrl=t_servo_ctrl,
                t_servo_sim=t_servo_sim
            )
        elif args.plot_type == 1:
            viz.visualize_less(
                ts_sim,
                t_total_sim
            )
        elif args.plot_type == 2:
            viz.visualize_rpy(
                ocp_solver.acados_ocp.model.name,
                ts_sim,
                t_total_sim
            )

    if args.save_data:
        file_path = args.file_path

        np.savez(
            file_path + f"nmpc_{type(nmpc).__name__}_model_{type(sim_nmpc).__name__}.npz",
            x=np.array(x_history),
            u=np.array(u_history)
        )

    return np.array(x_history), np.array(u_history)


if __name__ == "__main__":
    # Read command line arguments
    parser = argparse.ArgumentParser(description="Run the simulation of different NMPC models.")
    parser.add_argument(
        "model",
        type=int,
        help="The NMPC model to be simulated. "
             "Options: 0 (basic model), 1 (servo), "
             "2 (thrust), 3(servo+thrust), "
             "21 (servo+dist), 22 (servo+thrust+dist), "
             "91(no_servo_new_cost), 92(servo_old_cost), "
             "93(servo_diff), 94(servo+drag+dist), "
             "95 (servo+thrust+drag), 96 (servo+drag_param+dist).",
    )

    parser.add_argument(
        "-sim",
        "--sim_model",
        type=int,
        default=0,
        help="The simulation model. "
             "Options: 0 (default: servo+thrust), "
             "1 (servo+thrust+drag).",
    )

    parser.add_argument(
        "-p",
        "--plot_type",
        type=int,
        default=0,
        help="The type of plot. "
             "Options: 0 (default: full), 1 (less), 2 (only rpy)."
    )

    parser.add_argument(
        "-a",
        "--arch",
        type=str,
        default='qd',
        help="The robot's architecture. Options: bi, tri, qd (default)."
    )

    parser.add_argument(
        "--no_viz",
        action="store_true",
        help="Disable visualization after simulation. Note that this is different from the plot_type option, "
             "because plot_type also decides the simulation parameters."
    )

    parser.add_argument(
        "-s",
        "--save_data",
        action="store_true",
        help="Save simulation x and u data to file"
    )

    parser.add_argument(
        "--file_path",
        type=str,
        default=f"../../../../test/data/",
        help="Path to save the data file"
    )

    args = parser.parse_args()
    main(args)
