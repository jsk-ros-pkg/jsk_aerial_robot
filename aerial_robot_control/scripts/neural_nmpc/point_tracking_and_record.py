import sys
import time
import argparse
import numpy as np
import casadi as cs

from utils.data_utils import get_recording_dict_and_file, make_blank_dict, store_recording_data, write_recording_data
from utils.reference_utils import sample_random_target
from utils.geometry_utils import euclidean_dist
from utils.visualization_utils import draw_drone_simulation, initialize_drone_plotter
from config.configuration_parameters import SimpleSimConfig
from neural_controller import NeuralNMPC


def main(model_options, recording_options, sim_options, parameters):
    """
    Main function to run the NMPC simulation and recording.
    :param model_options: Options for the NMPC model.
    :param recording_options: Options for the recording.
    :param sim_options: Options for the simulation.
    :param parameters: Additional parameters for the simulation.
    """

    # ------------------------
    # TODO set these somewhere else
    MODEL_ID = 0
    version = None
    name = None
    # ------------------------
    
    # --- Initialize controller ---
    if version is not None and name is not None:
        # Load pre-trained model into NMPC object
        rtnmpc = NeuralNMPC.load_controller(model_options=model_options)
    else:
        # Create blank NMPC object
        rtnmpc = NeuralNMPC(model_options=model_options)

    # Solver
    ocp_solver = rtnmpc.get_ocp_solver()
    # Reference generator
    reference_generator = rtnmpc.get_reference_generator()

    # Recover some necessary variables from the NMPC object
    n_nodes = rtnmpc.n_nodes
    t_horizon = rtnmpc.t_horizon
    simulation_dt = rtnmpc.simulation_dt
    reference_over_sampling = 3     # TODO what is this?
    control_period = t_horizon / (n_nodes * reference_over_sampling)    # TODO what is this?

    # Sanity check: The optimization should be faster or equal than the duration of the optimization time step
    assert control_period <= t_horizon / n_nodes # TODO implement with real time duration?

    # --- Set initial state ---
    if parameters["initial_state"] is None:
        state_curr = [0.0, 0.0, 0.0] + [1, 0, 0, 0] + [0, 0, 0] + [0, 0, 0]
    else:
        state_curr = parameters["initial_state"]
    rtnmpc.set_state(state_curr)
    state_curr_sim = state_curr.copy()

    # --- Set target states ---
    if parameters["preset_targets"] is not None:
        targets = parameters["preset_targets"]
    else:
        targets = sample_random_target(np.array(state_curr[:3]), sim_options["world_radius"],
                                       aggressive=recording_options["aggressive"])
    targets_reached = [False for _ in targets]

    # --- Prepare recording ---
    recording = recording_options["recording"]
    if recording:
        state_dim = state_curr.shape[0]
        # TODO: create empty or get preloaded dict and filepath to store
        rec_dict, rec_file = get_recording_dict_and_file(recording_options, state_dim, sim_options)

        if parameters["real_time_plot"]:
            parameters["real_time_plot"] = False
            print("Turned off real time plot during recording mode.")

    # --- Real time plot ---
    # Real time plot params TODO set elsewhere
    plot_sim_traj = False
    quad_trajectory = np.array(state_curr).reshape(1, -1)
    
    # Generate necessary art pack for real time plot
    if parameters["real_time_plot"]:
        real_time_art_pack = initialize_drone_plotter(n_props=n_forward_props, quad_rad=my_quad.x_f,
                                                      world_rad=world_radius)
    else:
        real_time_art_pack = None

    # --- Set up simulation ---    
    # Initialize variables for first iteration
    start_time = time.time()
    simulation_time = 0.0
    initial_guess = parameters["initial_guess"]
    state_pred = None
    u_cmd = None

    # Cap simulation if emergency recovery is needed
    n_iteration_count = 0
    
    # --------- Targets loop ---------
    print("Targets reached: ", end='')
    while False in targets_reached and (time.time() - start_time) < parameters["max_sim_time"]:
        # --- Set current target ---
        current_target_idx = np.where(targets_reached == False)[0][0]
        current_target = targets[current_target_idx]
        current_target_reached = False

        # ---------- Reference ----------
        # TODO sim_nmpc.py has a Reference, here are targets. Decide which makes more sense
        # Reference in Input via allocation matrix - makes sense if we don't know model in the first place?
        # Alternative is setting yref in solver, i.e. the targets directly.
        # === TODO implement this idea from ml-casadi repo? ===
        #  Transform velocity to body frame
        v_b = v_dot_q(ref[7:10], quaternion_inverse(ref[3:7]))
        # =====================================================
        xr, ur = reference_generator.compute_trajectory(target_xyz=current_target[:3], target_rpy=current_target[3:])

        # =====================================================
        # TODO this was set by Jinjie (together with parameters) in most inner loop but by RTNMPC authors in outer loop
        # In rtnmpc they just set the same ref for all state
        # Jinjie updates ref in each time step
        # Potentially in dedicated function that updates/sets the solver? 
        for j in range(rtnmpc.N):
            yr = np.concatenate((xr[j, :], ur[j, :]))
            rtnmpc.acados_ocp_solver.set(j, "yref", yr)

        yr = xr[ocp_solver.N, :]
        rtnmpc.acados_ocp_solver.set(rtnmpc.N, "yref", yr)
        # =====================================================

        # Track targets in solver
        rtnmpc.track(targets) # TODO implement

        # --- Set initial guess ---
        # Provide a new initial guess when changing target
        # TODO fix initial guess? not very sensible rn
        if initial_guess is None:
            initial_guess = rtnmpc.optimize()

        # --------- NMPC loop ---------
        while not current_target_reached and (time.time() - start_time) < parameters["max_sim_time"]:

            # --- Emergency recovery --- (quad controller gone out of control lol)
            if np.any(state_curr[7:10] > 14) or n_iteration_count > 100: # TODO why check quaternions to be > 14?
                print("===== Emergency recovery triggered!!! =====")
                print("Iteration: ", n_iteration_count, " | Current state: ", state_curr)
                n_iteration_count = 0
                rtnmpc.set_state(current_target)

            # --- Get current state ---
            state_curr = rtnmpc.get_state()

            # --- Record time, target, current state and last optimized input ---
            if recording:
                rec_dict["timestamp"] = np.append(rec_dict["timestamp"], time.time() - start_time)
                rec_dict["target"] = np.append(rec_dict["target"], current_target[np.newaxis, :], axis=0)
                rec_dict["state_in"] = np.append(rec_dict["state_in"], state_curr.T, axis=0)
                if simulation_time != 0.0:
                    store_recording_data(rec_dict, state_curr.T, state_pred, u_cmd, simulation_time)

            # --- Optimize control input ---
            # Compute control feedback and take the first action
            u_cmd = rtnmpc.run_optimization(state_curr)
            # Select first input (one for each motor) - MPC applies only first optimized input to the plant
            ref_u = np.squeeze(np.array(u_cmd[:4]))

            # --- Check for constraints TODO should be done somewhere else ---
            if np.any(u_cmd > my_quad.max_input_value) or np.any(u_cmd < my_quad.min_input_value):
                print("===== MPC constraints were violated!!! =====")
                print("Iteration: ", n_iteration_count, " | Current state: ", state_curr, " | Input: ", u_cmd)

            # --- Reset initial guess ---
            initial_guess = rtnmpc.reshape_input_sequence(u_cmd)
            # TODO understand: "Save initial guess for future optimization. It is a time-shift of the current optimized variables"
            initial_guess = np.array(cs.vertcat(initial_guess[1:, :], cs.DM.zeros(4).T))


            if recording:
                # Integrate first input. Will be used as nominal model prediction during next save
                state_pred, _ = rtnmpc.forward_prop(np.squeeze(state_curr), u_cmd=u_cmd[:4],
                                                  t_horizon=control_period, use_gp=False)
                state_pred = state_pred[-1, :]

            # --- Plot realtime ---
            if parameters["real_time_plot"]:
                state_traj_sim = rtnmpc.simulate(state_curr, u_cmd, t_horizon=t_horizon)
                draw_drone_simulation(real_time_art_pack, quad_trajectory, my_quad, targets,
                                      targets_reached, x_sim, x_int, state_pred_horizon, follow_quad=False)

            # --- Simulate forward ---
            simulation_time = 0.0
            while simulation_time < control_period:
                # Simulation runtime (inner loop)
                simulation_time += simulation_dt

                # Simulate
                state_curr_sim = rtnmpc.simulate(state_curr_sim, u_cmd)

                # Target check
                if euclidean_dist(current_target[:3], state_curr[:3], thresh=0.05):
                    print("*", end='')
                    sys.stdout.flush()

                    # Check out data immediately as new target will be optimized in next step
                    if recording and len(rec_dict['state_in']) > len(rec_dict['input']):
                        state_pred, _ = rtnmpc.forward_prop(np.squeeze(state_curr), u_cmd=u_cmd[:4], t_horizon=simulation_time,
                                                      use_gp=False)
                        state_pred = state_pred[-1, :]
                        store_recording_data(rec_dict, rtnmpc.get_state().T, state_pred, u_cmd, simulation_time)

                    # Reset optimization count and time -> Trigger new optimization for next target in next dt
                    n_iteration_count = 0
                    simulation_time = 0.0

                    # Mark current target as reached
                    current_target_reached = True
                    targets_reached[current_target_idx] = True

                    # Remove initial guess
                    initial_guess = None

                    # Generate new target
                    if parameters["preset_targets"] is None:
                        new_target = sample_random_target(state_curr[:3], sim_options["world_radius"],
                                                          aggressive=recording_options["aggressive"])
                        targets.extend(new_target)
                        targets_reached.append(False)

                    # Reset PID integral and past errors
                    rtnmpc.reset() # TODO not implemented!! (not even in RTNMPC repo :D)

                    break

            # --- Set break condition ---
            n_iteration_count += 1

            # TODO what happens here?
            if parameters["real_time_plot"]:
                quad_trajectory = np.append(quad_trajectory, state_curr.T, axis=0)
                if len(quad_trajectory) > 300:
                    quad_trajectory = np.delete(quad_trajectory, 0, 0)

        # --- Save data ---
        # Current target reached!
        if recording:
            # Write data to file
            write_recording_data(rec_dict, rec_file)
            # Reset storage
            rec_dict = make_blank_dict(state_dim)


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
        "recording_options": {
            "is_recording": args.recording,
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
        "parameters": {
            "preset_targets": None,
            "initial_state": None,
            "initial_guess": None,
            "acados_options": acados_config
        }
    }

    main(**run_options)
    """
if __name__ == '__main__':

    acados_config = {
        "solver_type": "SQP",
        "terminal_cost": True
    }
    
    run_options = {
        "model_options": {
            "arch_type": "tilt_qd",
            "model_name": "test_model",
            "reg_type": "mlp"
        },
        "recording_options": {
            "is_recording": True,
            "dataset_name": "test_dataset",
            "split": "train",
            "aggressive": True  # TODO for now always use aggressive targets
        },
        "sim_options": {
            "disturbances": SimpleSimConfig.simulation_disturbances,
            "real_time_plot": SimpleSimConfig.custom_sim_gui,
            "max_sim_time": "300",
            "world_radius": 3
        },
        "parameters": {
            "preset_targets": None,
            "initial_state": None,
            "initial_guess": None,
            "acados_options": acados_config
        }
    }

    main(**run_options)
    """