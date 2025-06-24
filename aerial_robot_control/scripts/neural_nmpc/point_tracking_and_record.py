import sys, os
import time
import argparse
import numpy as np

from utils.data_utils import get_recording_dict_and_file, make_blank_dict, write_recording_data
from utils.reference_utils import sample_random_target
from utils.geometry_utils import euclidean_dist
from utils.visualization_utils import initialize_plotter, draw_robot, animate_robot
from config.configurations import SimpleSimConfig
from neural_controller import NeuralNMPC


np.random.seed(345)  # Set seed for reproducibility

def main(model_options, solver_options, dataset_options, sim_options, run_parameters):
    """
    Main function to run the NMPC simulation and recording.
    :param model_options: Options for the NMPC model.
    :param dataset_options: Options for the recording.
    :param sim_options: Options for the simulation.
    :param run_parameters: Additional parameters for the simulation.
    """

    # ------------------------
    # TODO set these somewhere else
    # Model options
    # model_options.update({
    #     "MODEL_ID": 0,
    #     "version": 1,
    #     "name": "test_model"
    # })
    T_sim = 0.005  # or 0.001
    # ------------------------

    # --- Initialize controller ---
    # if version is not None and name is not None:
    if True:
        # Create blank NMPC object
        rtnmpc = NeuralNMPC(model_options=model_options, solver_options=solver_options,
                            sim_options=sim_options, T_sim=T_sim)

    # Solver
    ocp_solver = rtnmpc.ocp_solver
    sim_solver = rtnmpc.sim_solver

    # Reference generator
    reference_generator = rtnmpc.get_reference_generator()

    # Recover some necessary variables from the NMPC object
    nx = rtnmpc.acados_model.x.shape[0]
    nu = rtnmpc.acados_model.u.shape[0]
    N = rtnmpc.N
    T_horizon = rtnmpc.T_horizon
    T_samp = rtnmpc.T_samp  # Time step for the control loop
    T_sim = rtnmpc.T_sim    # Simulation dt
    # reference_over_sampling = 1     # TODO what is this?
    # control_period = T_horizon / (N * reference_over_sampling)    # The time period between two control inputs

    # Sanity check: The optimization should be faster or equal than the duration of the optimization time step
    assert T_samp <= T_horizon / N
    assert T_samp >= T_sim

    # --- Set initial state ---
    if run_parameters["initial_state"] is None:
        # state = [p, v, q, w, (a and or t and or ds)]
        state_curr = np.zeros(nx)
        state_curr[6] = 1.0     # Real quaternion
    else:
        state_curr = run_parameters["initial_state"]
    state_curr_sim = state_curr.copy()

    # --- Set target states ---
    if run_parameters["preset_targets"] is not None:
        targets = run_parameters["preset_targets"]
    else:
        targets = sample_random_target(np.array(state_curr[:3]), sim_options["world_radius"],
                                       aggressive=run_parameters["aggressive"])
    targets_reached = np.array([False for _ in targets])

    # --- Prepare recording ---
    recording = run_parameters["recording"]
    if recording:
        # Create an empty dict or get a pre-recorded dict and filepath to store
        # TODO actually able to use a pre-recorded dict? And if so make it overwriteable
        model_options["state_dim"] = nx
        model_options["control_dim"] = nu
        ds_name = model_options["nmpc_type"] + "_" + dataset_options["ds_name_suffix"]
        rec_dict, rec_file = get_recording_dict_and_file(ds_name, model_options, sim_options, solver_options, targets[0].size)

        if run_parameters["real_time_plot"]:
            run_parameters["real_time_plot"] = False
            print("Turned off real time plot during recording mode.")

    # --- Real time plot ---
    # Real time plot params TODO set elsewhere
    plot_sim_traj = False
    animation = run_parameters["save_animation"]
    
    
    # Generate necessary art pack for real time plot
    if run_parameters["real_time_plot"]:
        art_pack = initialize_plotter(world_rad=sim_options["world_radius"], n_properties=N)
        trajectory_history = state_curr[np.newaxis, :]
        rotor_positions = rtnmpc.get_rotor_positions()
    else:
        art_pack = None

    # --- Set up simulation ---
    # state_pred = None
    u_cmd = None

    # Cap simulation if emergency recovery is needed
    i = 0; j = 0
    t_now = 0.0             # Total virtual time in seconds
    
    # --------- Targets loop ---------
    print("Targets reached: ", end='')
    while False in targets_reached:
        # --- Set current target ---
        current_target_idx = np.where(targets_reached == False)[0][0]
        current_target = targets[current_target_idx]
        current_target_reached = False

        # # ---------- Reference ----------
        # # TODO sim_nmpc.py has a Reference, here are targets. Decide which makes more sense
        # # Reference in Input via allocation matrix - makes sense if we don't know model in the first place?
        # # Alternative is setting yref in solver, i.e. the targets directly.
        # # === TODO implement this idea from ml-casadi repo? ===
        # #  Transform velocity to body frame
        # v_b = v_dot_q(ref[7:10], quaternion_inverse(ref[3:7]))
        # # =====================================================
        xr, ur = reference_generator.compute_trajectory(target_xyz=current_target[:3], target_rpy=current_target[6:9])

        # # =====================================================
        # # TODO this was set by Jinjie (together with parameters) in most inner loop but by RTNMPC authors in outer loop
        # # In rtnmpc they just set the same ref for all state
        # # Jinjie updates ref in each time step
        # # Potentially in dedicated function that updates/sets the solver? 
        # for j in range(rtnmpc.N):
        #     yr = np.concatenate((xr[j, :], ur[j, :]))
        #     rtnmpc.acados_ocp_solver.set(j, "yref", yr)

        # yr = xr[ocp_solver.N, :]
        # rtnmpc.acados_ocp_solver.set(rtnmpc.N, "yref", yr)
        # # =====================================================

        # Track targets in solver
        rtnmpc.track(xr, ur) # TODO implement

        # --- Set initial guess ---
        # TODO set initial guess to prev iteration?
        # TODO Provide a new initial guess when changing target

        # --------- NMPC loop ---------
        global_comp_time = time.time()
        while not current_target_reached:
            

            # --- Emergency recovery --- (quad controller gone out of control lol)
            if np.any(state_curr[7:10] > 14) or i > 500: # TODO why check quaternions to be > 14?
                print("===== Emergency recovery triggered!!! =====")
                print("Iteration: ", i, " | Current state: ", state_curr)
                i = 0
                ocp_solver.set("x", np.pad(current_target, (0,nx - current_target.size)))
                sim_solver.set("x", np.pad(current_target, (0,nx - current_target.size)))

            # --- Get current state ---
            # state_curr = ocp_solver.get(0, "x")
            if u_cmd is not None:
                # Basically don't overwrite initial state
                state_curr = sim_solver.get("x")
            else:
                # Set initial state in solver
                sim_solver.set("x", state_curr)

            # --- Optimize control input ---
            # Compute control feedback and take the first action
            comp_time = time.time()
            try:
                u_cmd = ocp_solver.solve_for_x0(state_curr)
            except Exception as e:
                print(f"Round {i}: acados ocp_solver returned status {ocp_solver.status}. Exiting.")
                # TODO try to recover from this
                raise e
            comp_time = time.time() - comp_time

            # --- Sanity check constraints ---
            rtnmpc.check_constraints(u_cmd)

            # --- Record time, target, current state and last optimized input ---
            if recording:
                rec_dict["timestamp"] = np.append(rec_dict["timestamp"], t_now)
                rec_dict["dt"] = np.append(rec_dict["dt"], t_now - rec_dict["timestamp"][-2] if len(rec_dict["timestamp"]) > 1 else 0.0)
                rec_dict["comp_time"] = np.append(rec_dict["comp_time"], comp_time)
                rec_dict["target"] = np.append(rec_dict["target"], current_target[np.newaxis, :], axis=0)
                rec_dict["state_in"] = np.append(rec_dict["state_in"], state_curr[np.newaxis, :], axis=0)
                rec_dict["control"] = np.append(rec_dict["control"], u_cmd[np.newaxis, :], axis=0)

            # # Select first input (one for each motor) - MPC applies only first optimized input to the plant
            # ref_u = np.squeeze(np.array(u_cmd[:4]))

            # # --- Check for constraints TODO should be done somewhere else ---
            # if np.any(u_cmd > my_quad.max_input_value) or np.any(u_cmd < my_quad.min_input_value):
            #     print("===== MPC constraints were violated!!! =====")
            #     print("Iteration: ", i, " | Current state: ", state_curr, " | Input: ", u_cmd)

            # # --- Reset initial guess ---
            # initial_guess = rtnmpc.reshape_input_sequence(u_cmd)
            # # TODO understand: "Save initial guess for future optimization. It is a time-shift of the current optimized variables"
            # initial_guess = np.array(cs.vertcat(initial_guess[1:, :], cs.DM.zeros(4).T))


            # if recording:
            #     # Integrate first input. Will be used as nominal model prediction during next save
            #     state_pred, _ = rtnmpc.forward_prop(np.squeeze(state_curr), u_cmd=u_cmd[:4],
            #                                       T_horizon=control_period, use_gp=False)
            #     state_pred = state_pred[-1, :]

            # --- Plot realtime ---
            if run_parameters["real_time_plot"]:
                trajectory_pred = rtnmpc.simulate_trajectory()
                draw_robot(art_pack, targets, targets_reached, state_curr,
                              trajectory_pred, trajectory_history,
                              rotor_positions, follow_robot=False, 
                              animation=animation)

            # --- Simulate forward ---
            # Simulate with the optimized input until the next time step of the control period is reached
            # Note: Pretend to run simulation in parallel to the control loop
            # i.e., the control loop has no effect on the simulation loop execution times
            # Simply trigger new control optimization after simulating for T_samp seconds
            simulation_time = 0.0
            j = 0
            state_curr_sim = state_curr.copy()  # TODO kind of redundant
            while simulation_time <= T_samp:
                # Simulation runtime (inner loop)
                simulation_time += T_sim
                # --- Increment virutal time ---
                # ASSUMPTION: Simulation time is exactly euqal to real time
                # i.e., the simulation has a zero runtime
                # This is somewhat realistic since in the real machine
                # the simulation (i.e. measurement + estimation) is run
                # in parallel to the real-time control loop.
                # Increment global time at every simulation step since the 
                # control loop runs in parallel and is assumpted to be idle at some times
                t_now += T_sim

                # Simulate
                try:
                    state_curr_sim = sim_solver.simulate(x=state_curr_sim, u=u_cmd)
                except Exception as e:
                    print(f"Round {i}.{j}: acados ocp_solver returned status {sim_solver.status}. Exiting.")
                    # TODO try to recover from this
                    raise e

                # Target check
                if euclidean_dist(current_target[:3], state_curr_sim[:3], thresh=0.05):
                    print("*", end='')
                    sys.stdout.flush()

                    # Check out data immediately as new target will be optimized in next step
                    # if recording and len(rec_dict['state_in']) > len(rec_dict['control']):
                        # state_pred, _ = rtnmpc.forward_prop(np.squeeze(state_curr_sim), u_cmd=u_cmd[:4], T_horizon=simulation_time,
                        #                               use_gp=False)
                        # state_pred = state_pred[-1, :]
                        # -> log pred state
                        # store_recording_data(rec_dict, state_curr_sim, ocp_solver.get(0, "x"))

                    # Reset optimization count and time -> Trigger new optimization for next target in next dt
                    i = 0; j = 0
                    simulation_time = 0.0

                    # Mark current target as reached
                    current_target_reached = True
                    targets_reached[current_target_idx] = True

                    # Remove initial guess
                    initial_guess = None

                    # Generate new target
                    if run_parameters["preset_targets"] is None:
                        new_target = sample_random_target(state_curr_sim[:3], sim_options["world_radius"],
                                                          aggressive=run_parameters["aggressive"])
                        targets = np.append(targets, new_target, axis=0)
                        targets_reached = np.append(targets_reached, False)
                    break
                # --- Increment simulation step ---
                j += 1
            # --- Increment control step ---
            i += 1

            # --- Record out data ---
            if recording:
                state_pred = ocp_solver.get(1, "x") # Predicted state by the controller at next sampling time
                rec_dict["state_out"] = np.append(rec_dict["state_out"], state_curr_sim[np.newaxis, :], axis=0)
                rec_dict["state_pred"] = np.append(rec_dict["state_pred"], state_pred[np.newaxis, :], axis=0)

                error = state_curr_sim - state_pred
                rec_dict["error"] = np.append(rec_dict["error"], error[np.newaxis, :], axis=0)

            # --- Log trajectory for real-time plot ---
            if run_parameters["real_time_plot"]:
                trajectory_history = np.append(trajectory_history, state_curr_sim[np.newaxis, :], axis=0)
                if len(trajectory_history) > 300:
                    trajectory_history = np.delete(trajectory_history, obj=0, axis=0) # Delete first logged state
        
            # --- Break condition for the inner loop ---
            if t_now >= sim_options["max_sim_time"]:
                break

        # Current target was reached!
        # --- Save data ---
        if recording:
            write_recording_data(rec_file, rec_dict)
            rec_dict = make_blank_dict(targets[0].size, nx, nu)

        # --- Break condition for the outer loop ---
        if t_now >= sim_options["max_sim_time"]:
            break

        print(f"Computation time for target {current_target_idx}: {time.time() - global_comp_time}")

    # --- Create video ---
    if animation:
        print("-------------- Saving Animation as video --------------")
        dir_path = os.path.dirname(os.path.abspath(__file__))
        counter = 1
        while True:
            file_name = f"video/robot_animation_{str(counter).zfill(3)}.mp4"
            file_path = os.path.join(dir_path, file_name)
            if not os.path.exists(file_path):
                break
            counter += 1
        print(f"Directory: {file_path}")
        animate_robot(file_path)

"""
if __name__ == '__main__':

    parser = argparse.ArgumentParser()

    parser.add_argument("--arch_type", type=str, default="tilt_qd",
                        help="Type of the aerial robot used. " \
                             "Options: tilt_bi, tilt_tri, (default) tilt_qd, fix_qd")

    parser.add_argument("--model_name", type=str, default="",
                        help="Name of the regression model.")

    parser.add_argument("--model_type", type=str, default="gp", choices=["gp", "rdrv"],
                        help="Type of the regression model (GP or RDRv linear).")

    parser.add_argument("--recording", dest="recording", action="store_true",
                        help="Set this flag to enable recording mode.")
    parser.set_defaults(recording=False)

    parser.add_argument("--dataset_name", type=str, default="simplified_sim_dataset",
                        help="Name for the generated dataset.")

    parser.add_argument("--test_split", dest="test_split", action="store_true",
                        help="Set this flag to use this dataset as test split; " \
                             "by default used as training split.")
    parser.set_defaults(test_split=False)

    parser.add_argument("--simulation_time", type=float, default=300,
                        help="Total duration of the simulation in seconds.")
    
    parser.add_argument("--world_radius", type=float, default=3,
                        help="Radius of the simulation environment in meters inside which " \
                             "the robot performs randomly sampled maneuvers. Default: 3 meters")

    args = parser.parse_args()

    np.random.seed(123 if args.test_split else 456)

    acados_config = {
        "solver_type": "SQP",
        "terminal_cost": True
    }

    split = "test" if args.test_split else "train"

    run_options = {
        "model_options": {
            "arch_type": args.arch_type,
            "model_name": args.model_name,
            "reg_type": args.model_type
        },
        "dataset_options": {
            "recording": args.recording,
            "dataset_name": args.dataset_name,
            "split": split,
            "aggressive": True  # TODO for now always use aggressive targets
        },
        "sim_options": {
            "disturbances": SimpleSimConfig.simulation_disturbances,
            "real_time_plot": SimpleSimConfig.custom_sim_gui,
            "max_sim_time": args.simulation_time,
            "world_radius": args.world_radius
        },
        "run_parameters": {
            "preset_targets": None,
            "initial_state": None,
            "initial_guess": None,
            "acados_options": acados_config
        }
    }

    main(**run_options)
    """
if __name__ == '__main__':
    
    options = {
        "model_options": {
            "model_name": "standard_neural_nmpc",
            "arch_type": "qd", # or "bi" or "tri"
            "nmpc_type": "NMPCTiltQdServo",
            #    NMPCFixQdAngvelOut
            #    NMPCFixQdThrustOut
            #    NMPCTiltQdNoServo
            #    NMPCTiltQdServo
            #    NMPCTiltQdServoDist
            #    NMPCTiltQdServoImpedance
            #    NMPCTiltQdServoThrustDist
            #    NMPCTiltQdServoThrustImpedance
            #    NMPCTiltTriServo
            #    NMPCTiltBiServo
            #    NMPCTiltBi2OrdServo
            #    MHEWrenchEstAccMom
            "use_mlp": False
        },
        "solver_options": {
            "solver_type": "PARTIAL_CONDENSING_HPIPM",  # TODO actually implement this
            "terminal_cost": True
        },
        "dataset_options": {
            "ds_name_suffix": "simple_dataset"
        },
        "sim_options": {
            "disturbances": SimpleSimConfig.disturbances,   # TODO use all disturbances in environment
            "max_sim_time": 60,
            "world_radius": 3,
        },
        "run_parameters": {
            "preset_targets": None,
            "initial_state": None,
            "initial_guess": None,
            "aggressive": True,  # TODO for now always use aggressive targets
            "recording": True,
            "real_time_plot": False,
            "save_animation": False,
        }
    }

    main(**options)
