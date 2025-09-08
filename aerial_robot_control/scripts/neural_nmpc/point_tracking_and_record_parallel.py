import os
import time
import numpy as np
import matplotlib.pyplot as plt

from sim_environment.forward_prop import init_forward_prop, forward_prop
from sim_environment.disturbances import apply_cog_disturbance, apply_motor_noise
from utils.data_utils import get_recording_dict_and_file, make_blank_dict, write_recording_data
from utils.model_utils import set_approximation_params, set_temporal_states_as_params
from utils.reference_utils import sample_random_target
from utils.geometry_utils import unit_quaternion, euclidean_dist
from utils.visualization_utils import initialize_plotter, draw_robot, animate_robot, plot_trajectory, plot_disturbances
from config.configurations import EnvConfig
from neural_controller import NeuralNMPC


def main(model_options, solver_options, dataset_options, sim_options, run_options):
    """
    Main function to run the NMPC simulation loop and label recording.
    :param model_options: Options for the NMPC model.
    :param dataset_options: Options for the recording.
    :param sim_options: Options for the simulation.
    :param run_options: Additional parameters for the simulation.
    """
    np.random.seed(sim_options["seed"])  # Set seed for reproducibility

    # ------------------------
    # TODO set these somewhere else
    # Model options
    # model_options.update({
    #     "MODEL_ID": 0,
    #     "version": 1,
    #     "name": "test_model"
    # })
    T_sim = 0.005  # or 0.001
    T_prop_step = 0.001
    # ------------------------

    # --- Initialize controller ---
    model_options["only_use_nominal"] = True
    rtnmpc_nominal = NeuralNMPC(
        model_options=model_options, solver_options=solver_options, sim_options=sim_options, run_options=run_options
    )
    model_options["only_use_nominal"] = False
    model_options["minus"] = True
    rtnmpc_minus_neural = NeuralNMPC(
        model_options=model_options, solver_options=solver_options, sim_options=sim_options, run_options=run_options
    )
    model_options["minus"] = False
    rtnmpc_plus_neural = NeuralNMPC(
        model_options=model_options, solver_options=solver_options, sim_options=sim_options, run_options=run_options
    )

    ocp_solver_nominal = rtnmpc_nominal.ocp_solver
    ocp_solver_minus_neural = rtnmpc_minus_neural.ocp_solver
    ocp_solver_plus_neural = rtnmpc_plus_neural.ocp_solver

    reference_generator = rtnmpc_nominal.get_reference_generator()

    # Recover some necessary variables from the NMPC object
    nx = rtnmpc_nominal.acados_model.x.shape[0]
    nu = rtnmpc_nominal.acados_model.u.shape[0]
    N = rtnmpc_nominal.N
    T_horizon = rtnmpc_nominal.T_horizon
    T_samp = rtnmpc_nominal.T_samp  # Time step for the control loop
    T_step = rtnmpc_nominal.T_step  # Time step in NMPC (= T_horizon / N)
    # reference_over_sampling = 1     # TODO what is this?
    # control_period = T_horizon / (N * reference_over_sampling)    # The time period between two control inputs

    # Sanity check: The optimization should be faster or equal than the duration of the optimization time step
    assert T_samp <= T_horizon / N
    assert T_samp >= T_sim

    # --- Initialize simulation environment ---
    # Create sim solver for the extended model
    sim_solver_nominal = rtnmpc_nominal.create_acados_sim_solver(T_sim)
    sim_solver_minus_neural = rtnmpc_minus_neural.create_acados_sim_solver(T_sim)
    sim_solver_plus_neural = rtnmpc_plus_neural.create_acados_sim_solver(T_sim)

    # Undisturbed model for creating labels to train on
    dynamics_forward_prop_nominal, state_forward_prop_nominal, u_forward_prop_nominal = init_forward_prop(
        rtnmpc_nominal.nmpc
    )
    (
        dynamics_forward_prop_minus_neural,
        state_forward_prop_minus_neural,
        u_forward_prop_minus_neural,
    ) = init_forward_prop(rtnmpc_minus_neural.nmpc)
    dynamics_forward_prop_plus_neural, state_forward_prop_plus_neural, u_forward_prop_plus_neural = init_forward_prop(
        rtnmpc_plus_neural.nmpc
    )

    # --- Set initial state ---
    if run_options["initial_state"] is None:
        # state = [p, v, q, w, (a and/or t and/or ds)]
        state_init = np.zeros(nx)
        state_init[6] = 1.0  # Real part of quaternion
    else:
        state_init = run_options["initial_state"]

    state_curr_nominal = state_init.copy()
    state_curr_minus_neural = state_init.copy()
    state_curr_plus_neural = state_init.copy()

    state_curr_sim_nominal = state_curr_nominal.copy()
    state_curr_sim_minus_neural = state_curr_minus_neural.copy()
    state_curr_sim_plus_neural = state_curr_plus_neural.copy()

    # # --- Set up running history for temporal neural networks ---
    # if rtnmpc.use_mlp and "temporal" in rtnmpc.mlp_metadata["MLPConfig"]["model_name"]:
    #     delay = rtnmpc.mlp_metadata["MLPConfig"]["delay_horizon"]  # Delay as number of time steps into the past
    #     history = np.tile(np.append(state_curr, np.zeros((nu,))), (delay, 1))

    # --- Set target states ---
    if run_options["preset_targets"] is not None:
        targets = run_options["preset_targets"]
    else:
        targets = sample_random_target(
            np.array(state_curr_nominal[:3]),
            sim_options["world_radius"],
            aggressive=run_options["aggressive"],
            low_flight=run_options["low_flight_targets"],
        )
    targets_reached = np.array([False for _ in targets])

    # --- Prepare recording ---
    # recording = run_options["recording"]
    # if recording:
    #     # Create an empty dict or get a pre-recorded dict and filepath to store
    #     # TODO actually able to use a pre-recorded dict? And if so make it overwriteable
    #     model_options["state_dim"] = nx
    #     model_options["control_dim"] = nu
    #     model_options["include_quaternion_constraint"] = rtnmpc.nmpc.include_quaternion_constraint
    #     model_options["include_soft_constraints"] = rtnmpc.nmpc.include_soft_constraints
    #     model_options["nmpc_params"] = rtnmpc.nmpc.params
    #     if rtnmpc.use_mlp:
    #         model_options["delay_horizon"] = rtnmpc.mlp_metadata["MLPConfig"]["delay_horizon"]
    #     ds_name = model_options["nmpc_type"] + "_" + dataset_options["ds_name_suffix"]
    #     rec_dict, rec_file = get_recording_dict_and_file(
    #         ds_name, model_options, sim_options, solver_options, targets[0].size
    #     )

    #     if run_options["real_time_plot"]:
    #         run_options["real_time_plot"] = False
    #         print("Turned off real time plot during recording mode.")

    # --- Real time plot ---
    # Generate necessary art pack for real time plot
    # if run_options["real_time_plot"]:
    #     art_pack = initialize_plotter(world_rad=sim_options["world_radius"], n_properties=N)
    #     trajectory_history = state_curr[np.newaxis, :]
    #     rotor_positions = rtnmpc.get_rotor_positions()

    plot = run_options["plot_trajectory"]
    if plot:
        rec_dict = {
            "timestamp": np.zeros((0, 1)),
            "dt": np.zeros((0, 1)),
            "comp_time": np.zeros((0, 1)),
            "target": np.zeros((0, targets[0].size)),
            "state_in_nominal": np.zeros((0, nx)),
            "state_in_minus_neural": np.zeros((0, nx)),
            "state_in_plus_neural": np.zeros((0, nx)),
            "state_out_nominal": np.zeros((0, nx)),
            "state_out_minus_neural": np.zeros((0, nx)),
            "state_out_plus_neural": np.zeros((0, nx)),
            "state_prop_nominal": np.zeros((0, nx)),
            "state_prop_minus_neural": np.zeros((0, nx)),
            "state_prop_plus_neural": np.zeros((0, nx)),
            "control_nominal": np.zeros((0, nu)),
            "control_minus_neural": np.zeros((0, nu)),
            "control_plus_neural": np.zeros((0, nu)),
        }
        dist_dict = {"timestamp": np.zeros((0, 1))}
        if rtnmpc_nominal.nmpc.include_cog_dist_parameter:
            dist_dict["z_nominal"] = np.zeros((0, 1))
            dist_dict["z_minus_neural"] = np.zeros((0, 1))
            dist_dict["z_plus_neural"] = np.zeros((0, 1))
            dist_dict["cog_dist_nominal"] = np.zeros((0, 6))
            dist_dict["cog_dist_minus_neural"] = np.zeros((0, 6))
            dist_dict["cog_dist_plus_neural"] = np.zeros((0, 6))
        # if rtnmpc_nominal.nmpc.include_motor_noise_parameter:
        #     dist_dict["motor_noise"] = np.zeros((0, 8))
        # "drag": np.zeros((0, 0)),
        # "payload": np.zeros((0, 0)),

    # --- Set up simulation ---
    u_cmd_nominal = None
    u_cmd_minus_neural = None
    u_cmd_plus_neural = None
    i = 0
    j = 0
    t_now = 0.0  # Total virtual time in seconds

    # ---------- Targets loop ----------
    print("Targets reached:")
    while False in targets_reached:
        # --- Target ---
        current_target_idx = np.where(targets_reached == False)[0][0]
        current_target = targets[current_target_idx]
        current_target_reached = False

        # --- Reference ---
        # Compute reference for Input u with an allocation matrix - TODO still makes sense if we don't know model in the first place?
        # Alternative is setting the modular trajectory yref dynamically in control loop
        state_ref, control_ref = reference_generator.compute_trajectory(
            target_xyz=current_target[:3], target_rpy=current_target[6:9]
        )
        # Track reference in solver over horizon
        rtnmpc_nominal.track(state_ref, control_ref)
        rtnmpc_minus_neural.track(state_ref, control_ref)
        rtnmpc_plus_neural.track(state_ref, control_ref)

        # --------- NMPC loop ---------
        global_comp_time = time.time()
        while not current_target_reached:
            # --- Emergency recovery --- (quad controller gone out of control lol)
            if i > 500:
                print("===== Emergency recovery triggered!!! =====")
                print(f"Iteration: {i}")
                print(f"Euclidean dist nominal: {(current_target[:3] - state_curr_sim_nominal[:3]) ** 2}")
                print(f"Current state nominal: {state_curr_sim_nominal}")
                print(f"Euclidean dist minus neural: {(current_target[:3] - state_curr_sim_minus_neural[:3]) ** 2}")
                print(f"Current state minus neural: {state_curr_sim_minus_neural}")
                print(f"Euclidean dist plus neural: {(current_target[:3] - state_curr_sim_plus_neural[:3]) ** 2}")
                print(f"Current state plus neural: {state_curr_sim_plus_neural}")
                i = 0
                ocp_solver_nominal.set(0, "x", state_ref[-1, :])
                ocp_solver_minus_neural.set(0, "x", state_ref[-1, :])
                ocp_solver_plus_neural.set(0, "x", state_ref[-1, :])
                sim_solver_nominal.set("x", state_ref[-1, :])
                sim_solver_minus_neural.set("x", state_ref[-1, :])
                sim_solver_plus_neural.set("x", state_ref[-1, :])
                u_cmd_nominal = None
                u_cmd_minus_neural = None
                u_cmd_plus_neural = None
                state_curr_nominal = state_ref[-1, :]
                state_curr_minus_neural = state_ref[-1, :]
                state_curr_plus_neural = state_ref[-1, :]

            # --- Get current state ---
            if u_cmd_nominal is None:
                # If no command is available, use initial/last state
                sim_solver_nominal.set("x", state_curr_nominal)
            else:
                state_curr_nominal = sim_solver_nominal.get("x")
                rtnmpc_nominal.check_state_constraints(state_curr_nominal, i)

            if u_cmd_minus_neural is None:
                # If no command is available, use initial/last state
                sim_solver_minus_neural.set("x", state_curr_minus_neural)
            else:
                state_curr_minus_neural = sim_solver_minus_neural.get("x")
                rtnmpc_minus_neural.check_state_constraints(state_curr_minus_neural, i)

            if u_cmd_plus_neural is None:
                # If no command is available, use initial/last state
                sim_solver_plus_neural.set("x", state_curr_plus_neural)
            else:
                state_curr_plus_neural = sim_solver_plus_neural.get("x")
                rtnmpc_plus_neural.check_state_constraints(state_curr_plus_neural, i)

            # --- Initial guess ---
            # TODO set initial guess to prev iteration?
            # TODO Provide a new initial guess when changing target
            # initial_guess = rtnmpc.reshape_input_sequence(u_cmd)
            # TODO understand: "Save initial guess for future optimization. It is a time-shift of the current optimized variables"
            # initial_guess = np.array(cs.vertcat(initial_guess[1:, :], cs.DM.zeros(4).T))

            # --- Prepare neural model approximation ---
            # Optimization cycle
            # RTNMPC without model:    0.31 ms
            # RTNMPC with their model: 0.53 ms (without approximation) -> + 71%
            # Ours without model:      1.05 ms
            # Ours with our model:     3.69 ms (without approximation) -> + 250% [min]
            # Ours with their model:   1.56 ms (without approximation) -> + 48%  [min]
            # Ours with our model:     27.1 ms (without approximation) [4x64+full in]
            # Ours with their model:   17.1 ms (without approximation) [4x64+full in&out]

            # if rtnmpc.use_mlp and model_options["approximate_mlp"]:
            #     set_approximation_params(rtnmpc, ocp_solver)

            # # --- Prepare temporal neural network input ---
            # if rtnmpc.use_mlp and "temporal" in rtnmpc.mlp_metadata["MLPConfig"]["model_name"]:
            #     set_temporal_states_as_params(rtnmpc, ocp_solver, history, u_cmd)

            # --- Set parameters in OCP solver ---
            for j in range(ocp_solver_nominal.N + 1):
                ocp_solver_nominal.set(j, "p", rtnmpc_nominal.acados_parameters[j, :])
                ocp_solver_minus_neural.set(j, "p", rtnmpc_minus_neural.acados_parameters[j, :])
                ocp_solver_plus_neural.set(j, "p", rtnmpc_plus_neural.acados_parameters[j, :])

            ############################################################################################
            # --- Optimize control input ---
            # Compute control feedback and take the first action
            # comp_time = time.time()
            try:
                # acados wrapper to solve the OCP and get first control command from sequence
                u_cmd_nominal = ocp_solver_nominal.solve_for_x0(state_curr_nominal)
                u_cmd_minus_neural = ocp_solver_minus_neural.solve_for_x0(state_curr_minus_neural)
                u_cmd_plus_neural = ocp_solver_plus_neural.solve_for_x0(state_curr_plus_neural)
            except Exception as e:
                print(
                    f"Round {i}: acados ocp_solver returned status {ocp_solver_nominal.status}, \
                      {ocp_solver_minus_neural.status}, {ocp_solver_plus_neural.status}. Exiting."
                )
                raise e
            # comp_time = (time.time() - comp_time) * 1000  # in ms

            # --- Sanity check constraints ---
            rtnmpc_nominal.check_input_constraints(u_cmd_nominal, i)
            rtnmpc_minus_neural.check_input_constraints(u_cmd_minus_neural, i)
            rtnmpc_plus_neural.check_input_constraints(u_cmd_plus_neural, i)
            ############################################################################################

            # --- Running history for temporal neural networks ---
            # if rtnmpc.use_mlp and "temporal" in rtnmpc.mlp_metadata["MLPConfig"]["model_name"]:
            #     # Append current state and control to history for next iteration
            #     # Sorted from newest to oldest
            #     history = history[:-1, :]
            #     history = np.append(np.append(state_curr, u_cmd)[np.newaxis, :], history, axis=0)

            # --- Record time, target, current state and last optimized input ---
            if plot:  # or recording:
                rec_dict["timestamp"] = np.append(rec_dict["timestamp"], t_now)
                rec_dict["dt"] = np.append(  # -2 since current timestamp is just added on index -1
                    rec_dict["dt"], t_now - rec_dict["timestamp"][-2] if len(rec_dict["timestamp"]) > 1 else T_samp
                )
                # rec_dict["comp_time"] = np.append(rec_dict["comp_time"], comp_time)
                rec_dict["target"] = np.append(rec_dict["target"], current_target[np.newaxis, :], axis=0)
                rec_dict["state_in_nominal"] = np.append(
                    rec_dict["state_in_nominal"], state_curr_nominal[np.newaxis, :], axis=0
                )
                rec_dict["state_in_minus_neural"] = np.append(
                    rec_dict["state_in_minus_neural"], state_curr_minus_neural[np.newaxis, :], axis=0
                )
                rec_dict["state_in_plus_neural"] = np.append(
                    rec_dict["state_in_plus_neural"], state_curr_plus_neural[np.newaxis, :], axis=0
                )
                rec_dict["control_nominal"] = np.append(
                    rec_dict["control_nominal"], u_cmd_nominal[np.newaxis, :], axis=0
                )
                rec_dict["control_minus_neural"] = np.append(
                    rec_dict["control_minus_neural"], u_cmd_minus_neural[np.newaxis, :], axis=0
                )
                rec_dict["control_plus_neural"] = np.append(
                    rec_dict["control_plus_neural"], u_cmd_plus_neural[np.newaxis, :], axis=0
                )

            # --- Plot realtime ---
            # if run_options["real_time_plot"]:
            #     # Note: Simulation is without disturbance here !
            #     #########################################
            #     # TODO OVERTHINK THIS!!!
            #     state_traj = rtnmpc.simulate_trajectory(sim_solver, state_curr)
            #     #########################################
            #     draw_robot(
            #         art_pack,
            #         targets,
            #         targets_reached,
            #         state_curr,
            #         state_traj,
            #         trajectory_history,
            #         rotor_positions,
            #         follow_robot=False,
            #         animation=run_options["save_animation"],
            #     )

            # --- Simulate forward ---
            # Simulate with the optimized input until the next time step of the control period is reached
            # Note: Pretend to run simulation in parallel to the control loop
            # i.e., the control loop has no effect on the simulation loop execution times
            # Simply trigger new control optimization after simulating for T_samp seconds
            simulation_time = 0.0
            j = 0
            state_curr_sim_nominal = state_curr_nominal.copy()
            state_curr_sim_minus_neural = state_curr_minus_neural.copy()
            state_curr_sim_plus_neural = state_curr_plus_neural.copy()
            while simulation_time < T_samp:
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

                # --- Set disturbance forces as parameters ---
                # TODO only apply disturbance to simulation model for next time step
                if rtnmpc_nominal.nmpc.include_cog_dist_parameter:
                    apply_cog_disturbance(
                        rtnmpc_nominal,
                        sim_options["disturbances"]["cog_dist_factor"],
                        u_cmd_nominal,
                        state_curr_nominal,
                    )
                    apply_cog_disturbance(
                        rtnmpc_minus_neural,
                        sim_options["disturbances"]["cog_dist_factor"],
                        u_cmd_minus_neural,
                        state_curr_minus_neural,
                    )
                    apply_cog_disturbance(
                        rtnmpc_plus_neural,
                        sim_options["disturbances"]["cog_dist_factor"],
                        u_cmd_plus_neural,
                        state_curr_plus_neural,
                    )
                    if plot:
                        dist_dict["timestamp"] = np.append(dist_dict["timestamp"], t_now)
                        dist_dict["z_nominal"] = np.append(dist_dict["z_nominal"], state_curr_sim_nominal[2])
                        dist_dict["z_minus_neural"] = np.append(
                            dist_dict["z_minus_neural"], state_curr_sim_minus_neural[2]
                        )
                        dist_dict["z_plus_neural"] = np.append(
                            dist_dict["z_plus_neural"], state_curr_sim_plus_neural[2]
                        )
                        dist_dict["cog_dist_nominal"] = np.append(
                            dist_dict["cog_dist_nominal"],
                            rtnmpc_nominal.sim_acados_parameters[
                                np.newaxis,
                                rtnmpc_nominal.nmpc.cog_dist_start_idx : rtnmpc_nominal.nmpc.cog_dist_end_idx,
                            ],
                            axis=0,
                        )
                        dist_dict["cog_dist_minus_neural"] = np.append(
                            dist_dict["cog_dist_minus_neural"],
                            rtnmpc_minus_neural.sim_acados_parameters[
                                np.newaxis,
                                rtnmpc_minus_neural.nmpc.cog_dist_start_idx : rtnmpc_minus_neural.nmpc.cog_dist_end_idx,
                            ],
                            axis=0,
                        )
                        dist_dict["cog_dist_plus_neural"] = np.append(
                            dist_dict["cog_dist_plus_neural"],
                            rtnmpc_plus_neural.sim_acados_parameters[
                                np.newaxis,
                                rtnmpc_plus_neural.nmpc.cog_dist_start_idx : rtnmpc_plus_neural.nmpc.cog_dist_end_idx,
                            ],
                            axis=0,
                        )

                # Simulate
                try:
                    state_curr_sim_nominal = sim_solver_nominal.simulate(
                        x=state_curr_sim_nominal, u=u_cmd_nominal, p=rtnmpc_nominal.sim_acados_parameters
                    )
                    state_curr_sim_minus_neural = sim_solver_minus_neural.simulate(
                        x=state_curr_sim_minus_neural, u=u_cmd_minus_neural, p=rtnmpc_minus_neural.sim_acados_parameters
                    )
                    state_curr_sim_plus_neural = sim_solver_plus_neural.simulate(
                        x=state_curr_sim_plus_neural, u=u_cmd_plus_neural, p=rtnmpc_plus_neural.sim_acados_parameters
                    )
                except Exception as e:
                    # print(f"Round {i}.{j}: acados ocp_solver returned status {sim_solver_nominal.status}. Exiting.")
                    # TODO try to recover from this
                    raise e

                # Ensure unit quaternion
                state_curr_sim_nominal[6:10] = unit_quaternion(state_curr_sim_nominal[6:10])
                state_curr_sim_minus_neural[6:10] = unit_quaternion(state_curr_sim_minus_neural[6:10])
                state_curr_sim_plus_neural[6:10] = unit_quaternion(state_curr_sim_plus_neural[6:10])

                # Target check
                if euclidean_dist(current_target[:3], state_curr_sim_nominal[:3], thresh=0.075):
                    # Target reached!
                    current_target_reached = True
                    targets_reached[current_target_idx] = True

                    # NOTE: Break condition turned off to allow for complete simulation
                    # and therefore smooth trajectory
                    # Also, it makes no physical sense to jump to next control step immediately
                    # break
                # --- Increment simulation step ---
                j += 1
            # --- Increment control step ---
            if current_target_reached:
                i = 0
                j = 0
                simulation_time = 0.0

                # Remove initial guess
                # initial_guess = None

                # Generate new target
                if run_options["preset_targets"] is None:
                    new_target = sample_random_target(
                        state_curr_sim_nominal[:3],
                        sim_options["world_radius"],
                        aggressive=run_options["aggressive"],
                        low_flight=run_options["low_flight_targets"],
                    )
                    targets = np.append(targets, new_target, axis=0)
                    targets_reached = np.append(targets_reached, False)
            else:
                i += 1

            # --- Record out data ---
            if plot:  # or recording:
                # State after simulation
                rec_dict["state_out_nominal"] = np.append(
                    rec_dict["state_out_nominal"], state_curr_sim_nominal[np.newaxis, :], axis=0
                )
                rec_dict["state_out_minus_neural"] = np.append(
                    rec_dict["state_out_minus_neural"], state_curr_sim_minus_neural[np.newaxis, :], axis=0
                )
                rec_dict["state_out_plus_neural"] = np.append(
                    rec_dict["state_out_plus_neural"], state_curr_sim_plus_neural[np.newaxis, :], axis=0
                )

                ###################### OLD ###########################
                # if T_samp != T_step:
                #     raise ValueError("T_samp and T_step must be equal for prediction to make any sense since.")
                # NOTE this gets the state after T_step = 0.1 but the curr state is only passed for T_samp = 0.01
                # state_prop = ocp_solver.get(1, "x") # Predicted state by the controller at next sampling time
                ######################################################

                # Compute next state prediction through more precise and undisturbed integration
                state_prop_nominal = forward_prop(
                    dynamics_forward_prop_nominal,
                    state_forward_prop_nominal,
                    u_forward_prop_nominal,
                    state_curr_nominal[np.newaxis, :],
                    u_cmd_nominal[np.newaxis, :],
                    T_horizon=T_samp,
                    T_step=T_sim,  # T_prop_step,
                    num_stages=4,
                )
                state_prop_minus_neural = forward_prop(
                    dynamics_forward_prop_minus_neural,
                    state_forward_prop_minus_neural,
                    u_forward_prop_minus_neural,
                    state_curr_minus_neural[np.newaxis, :],
                    u_cmd_minus_neural[np.newaxis, :],
                    T_horizon=T_samp,
                    T_step=T_sim,  # T_prop_step,
                    num_stages=4,
                )
                state_prop_plus_neural = forward_prop(
                    dynamics_forward_prop_plus_neural,
                    state_forward_prop_plus_neural,
                    u_forward_prop_plus_neural,
                    state_curr_plus_neural[np.newaxis, :],
                    u_cmd_plus_neural[np.newaxis, :],
                    T_horizon=T_samp,
                    T_step=T_sim,  # T_prop_step,
                    num_stages=4,
                )
                state_prop_nominal = state_prop_nominal[-1, :]  # Get last predicted state
                state_prop_minus_neural = state_prop_minus_neural[-1, :]  # Get last predicted state
                state_prop_plus_neural = state_prop_plus_neural[-1, :]  # Get last predicted state
                rec_dict["state_prop_nominal"] = np.append(
                    rec_dict["state_prop_nominal"], state_prop_nominal[np.newaxis, :], axis=0
                )
                rec_dict["state_prop_minus_neural"] = np.append(
                    rec_dict["state_prop_minus_neural"], state_prop_minus_neural[np.newaxis, :], axis=0
                )
                rec_dict["state_prop_plus_neural"] = np.append(
                    rec_dict["state_prop_plus_neural"], state_prop_plus_neural[np.newaxis, :], axis=0
                )

            # --- Log trajectory for real-time plot ---
            # if run_options["real_time_plot"]:
            #     trajectory_history = np.append(trajectory_history, state_curr_sim[np.newaxis, :], axis=0)
            #     if len(trajectory_history) > 300:
            #         trajectory_history = np.delete(trajectory_history, obj=0, axis=0)

            # --- Break condition for the inner loop ---
            if t_now >= sim_options["max_sim_time"]:
                break

        # Current target was reached!
        # --- Save data ---
        # if recording:
        #     write_recording_data(rec_dict, rec_file)
        #     rec_dict = make_blank_dict(targets[0].size, nx, nu)

        # --- Break condition for the outer loop ---
        if t_now >= sim_options["max_sim_time"]:
            break

        print(f"Computation time for target {current_target_idx}: {time.time() - global_comp_time}")

    # End of simulation
    # if recording:
    #     print(f"Recording finished. Data saved to {rec_file}")

    # --- Create video ---
    # if run_options["save_animation"]:
    #     print("-------------- Saving animation as video --------------")
    #     dir_path = os.path.dirname(os.path.abspath(__file__))
    #     counter = 1
    #     while True:
    #         file_name = f"video/robot_animation_{str(counter).zfill(3)}.mp4"
    #         file_path = os.path.join(dir_path, file_name)
    #         if not os.path.exists(file_path):
    #             break
    #         counter += 1
    #     animate_robot(file_path)
    #     print(f"Saved in directory: {file_path}")

    # --- Plot simple trajectory ---
    if plot:  # and not recording:
        from utils.plot_trajectory_parallel import plot_trajectory_parallel

        plot_trajectory_parallel(
            rec_dict,
            rtnmpc_nominal,
            rtnmpc_minus_neural,
            rtnmpc_plus_neural,
            dist_dict=dist_dict,
            save=run_options["save_figures"],
        )
        # if rtnmpc.nmpc.include_cog_dist_parameter or rtnmpc.nmpc.include_motor_noise_parameter:
        #     plot_disturbances(dist_dict, save=run_options["save_figures"])
        plt.show()

    halt = 1


if __name__ == "__main__":
    main(
        EnvConfig.model_options,
        EnvConfig.solver_options,
        EnvConfig.dataset_options,
        EnvConfig.sim_options,
        EnvConfig.run_options,
    )
