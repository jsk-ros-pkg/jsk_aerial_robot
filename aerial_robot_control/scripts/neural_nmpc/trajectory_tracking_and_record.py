import os
import time
import numpy as np
import matplotlib.pyplot as plt

from sim_environment.sim_solver import create_acados_sim_solver, simulate_trajectory
from sim_environment.forward_prop import init_forward_prop, forward_prop
from sim_environment.disturbances import apply_cog_disturbance, apply_motor_noise
from utils.controller_utils import check_state_constraints, check_input_constraints, get_rotor_positions
from utils.data_utils import get_recording_dict_and_file, make_blank_dict, write_recording_data
from utils.model_utils import set_approximation_params, set_temporal_states_as_params
from utils.reference_utils import sample_random_target
from utils.geometry_utils import unit_quaternion, euclidean_dist
from utils.visualization_utils import initialize_plotter, draw_robot, animate_robot, plot_trajectory, plot_disturbances
from config.configurations import EnvConfig
from neural_controller_standalone import NeuralNMPC
import random


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

    rtnmpc = NeuralNMPC(
        model_options=model_options, solver_options=solver_options, sim_options=sim_options, run_options=run_options
    )

    ocp_solver = rtnmpc.get_ocp_solver()
    ocp_model = rtnmpc.get_acados_model()
    reference_generator = rtnmpc.get_reference_generator()

    # Recover some necessary variables from the NMPC object
    nx = ocp_model.x.shape[0]
    nu = ocp_model.u.shape[0]
    N = rtnmpc.N
    T_horizon = rtnmpc.T_horizon
    T_samp = rtnmpc.T_samp  # Time step for the control loop
    T_step = rtnmpc.T_step  # Time step in NMPC (= T_horizon / N)
    # reference_over_sampling = 1     # TODO what is this?
    # control_period = T_horizon / (N * reference_over_sampling)    # The time period between two control inputs

    # Sanity check: The optimization should be faster or equal than the duration of the optimization time step
    assert T_samp <= T_horizon / N
    assert T_samp >= T_sim

    # --- Initialize simulation environment ---
    if sim_options["use_real_world_simulator"]:
        # Use neural model trained on real world data as simulator
        temp = {
            "only_use_nominal": model_options["only_use_nominal"],
            "plus_neural": model_options["plus_neural"],
            "minus_neural": model_options["minus_neural"],
            "neural_model_instance": model_options["neural_model_instance"],
        }
        model_options["only_use_nominal"] = False
        model_options["plus_neural"] = True
        model_options["minus_neural"] = False
        model_options["neural_model_instance"] = "neuralmodel_082"  # 72"  #58"
        rtnmpc_sim = NeuralNMPC(
            model_options=model_options,
            solver_options=solver_options,
            sim_options=sim_options,
            run_options=run_options,
            use_as_simulator=True,
        )
        sim_model = rtnmpc_sim.get_acados_model()
        # Create sim solver from the neural-enhanced model
        sim_solver = create_acados_sim_solver(rtnmpc_sim, sim_model, T_sim)
        # Restore model options for metadata
        model_options.update(temp)
    elif sim_options["use_nominal_simulator"]:
        # Use nominal model as simulator
        temp = {"only_use_nominal": model_options["only_use_nominal"]}
        model_options["only_use_nominal"] = True
        rtnmpc_sim = NeuralNMPC(
            model_options=model_options,
            solver_options=solver_options,
            sim_options=sim_options,
            run_options=run_options,
            use_as_simulator=True,
        )
        sim_model = rtnmpc_sim.get_acados_model()
        # Create sim solver from the nominal model
        sim_solver = create_acados_sim_solver(rtnmpc_sim, sim_model, T_sim)
        # Restore model options for metadata
        model_options.update(temp)
    else:
        # Create sim solver
        sim_solver = create_acados_sim_solver(rtnmpc, ocp_model, T_sim)
    # Undisturbed model for creating labels to train on
    dynamics_forward_prop, state_forward_prop, u_forward_prop = init_forward_prop(rtnmpc)

    # --- Set initial state ---
    if run_options["initial_state"] is None:
        # state = [p, v, q, w, (a and/or t and/or ds)]
        state_curr = np.zeros(nx)
        state_curr[6] = 1.0  # Real part of quaternion
    else:
        state_curr = run_options["initial_state"]
    state_curr_sim = state_curr.copy()

    # --- Reference trajectories ---
    traj_list = run_options["trajectories"]

    # --- Set up running history for temporal neural networks ---
    if rtnmpc.use_mlp and "temporal" in rtnmpc.mlp_metadata["MLPConfig"]["model_name"]:
        delay = rtnmpc.mlp_metadata["MLPConfig"]["delay_horizon"]  # Delay as number of time steps into the past
        history = np.tile(np.append(state_curr, np.zeros((nu,))), (delay, 1))

    # --- Prepare recording ---
    recording = run_options["recording"]
    if recording:
        # Create an empty dict or get a pre-recorded dict and filepath to store
        # TODO actually able to use a pre-recorded dict? And if so make it overwriteable
        model_options["state_dim"] = nx
        model_options["control_dim"] = nu
        model_options["include_quaternion_constraint"] = rtnmpc.include_quaternion_constraint
        model_options["include_soft_constraints"] = rtnmpc.include_soft_constraints
        model_options["nmpc_params"] = rtnmpc.params
        if rtnmpc.use_mlp:
            model_options["delay_horizon"] = rtnmpc.mlp_metadata["MLPConfig"]["delay_horizon"]
        else:
            model_options["delay_horizon"] = 0
        ds_name = model_options["nmpc_type"] + "_" + dataset_options["ds_name_suffix"]
        rec_dict, rec_file = get_recording_dict_and_file(ds_name, model_options, sim_options, solver_options, nx)

        if run_options["real_time_plot"]:
            run_options["real_time_plot"] = False
            print("Turned off real time plot during recording mode.")

    # --- Real time plot ---
    # Generate necessary art pack for real time plot
    if run_options["real_time_plot"]:
        # TODO fix reference trajectory plotting
        art_pack = initialize_plotter(world_rad=sim_options["world_radius"], n_properties=N)
        trajectory_history = state_curr[np.newaxis, :]
        rotor_positions = get_rotor_positions(rtnmpc)

    plot = run_options["plot_trajectory"]
    if plot:
        rec_dict = make_blank_dict(nx, nx, nu)
        dist_dict = {"timestamp": np.zeros((0, 1))}
        if rtnmpc.include_cog_dist_parameter:
            dist_dict["z"] = np.zeros((0, 1))
            dist_dict["cog_dist"] = np.zeros((0, 6))
        if rtnmpc.include_motor_noise_parameter:
            dist_dict["motor_noise"] = np.zeros((0, 8))
        # "drag": np.zeros((0, 0)),
        # "payload": np.zeros((0, 0)),

    # --- Set up simulation ---
    u_cmd = None
    i = 0
    j = 0
    t_now = 0.0  # Total virtual time in seconds

    # ---------- Trajectory loop ----------
    T_takeoff = 5.0
    t_last = T_takeoff
    global_comp_time = time.time()
    print_trakeoff = True
    while t_now < sim_options["max_sim_time"]:
        # Get next reference
        traj = np.random.choice(traj_list)  # , p=[0.1, 0.3, 0.2, 0.1, 0.3])
        print("-----------------------------------")
        print(f"Tracking trajectory: {traj}")
        reached_init = False
        finished = False
        print_init = True
        print_track = True
        k = 0

        while not finished:
            # --- Reference ---
            state_ref = np.zeros((N + 1, nx))
            state_ref[:, 6] = 1.0
            control_ref = np.zeros((N, nu))
            for n in range(N + 1):
                # Pose reference
                # First takeoff, then selected trajectory
                if t_now < T_takeoff:
                    if print_trakeoff:
                        print("Taking off...")
                        print_trakeoff = False
                    pose_ref = reference_generator.get_pose_from_trajectory("smooth_takeoff", t_now, T_takeoff)
                    last_ref = pose_ref
                else:
                    if not reached_init:
                        if print_init:
                            print("Going to init pose...")
                            print_init = False
                        # Get init pose to fly from current position
                        pose_init = reference_generator.get_pose_from_trajectory(
                            traj, 0, run_options["trajectory_length"]
                        )
                        # Smooth transition to init pose of next trajectory
                        pose_ref = np.array(last_ref) + (np.array(pose_init) - np.array(last_ref)) / (
                            1 + 100 * np.exp(-(t_now - t_last) * 4)
                        )

                        if euclidean_dist(state_curr[0:3], pose_init[0:3]) < 0.1 or k > 5000:
                            reached_init = True
                            t_init = t_now
                            k = 0
                        else:
                            k += 1
                    else:
                        if print_track:
                            print("Init pose reached. Now tracking trajectory...")
                            print_track = False
                        pose_ref = reference_generator.get_pose_from_trajectory(
                            traj, t_now - t_init + n * T_step, run_options["trajectory_length"]
                        )
                        if n == 0 and pose_ref is not None:
                            # Store last reference to smoothly go to init pose of next trajectory
                            last_ref = pose_ref

                if pose_ref is None:
                    finished = True
                    reached_init = False
                    t_last = t_now
                    print(f"Tracking finished for {traj}!")
                    state_ref[n, :] = state_ref_const[0, :]
                    if n < N:
                        control_ref[n, :] = control_ref_const[0, :]
                    continue

                # Compute reference for Input u with an allocation matrix - TODO still makes sense if we don't know model in the first place?
                # Alternative is setting the modular trajectory yref dynamically in control loop
                state_ref_const, control_ref_const = reference_generator.compute_trajectory(
                    target_xyz=pose_ref[0:3], target_rpy=pose_ref[3:6]
                )

                # Append reference
                state_ref[n, :] = state_ref_const[0, :]
                if n < N:
                    control_ref[n, :] = control_ref_const[0, :]
            # Track reference in solver over horizon
            rtnmpc.track(ocp_solver, state_ref, control_ref)

            # --- Get current state ---
            state_curr = state_curr_sim.copy()

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
            if rtnmpc.use_mlp and model_options["approximate_mlp"]:
                set_approximation_params(rtnmpc, ocp_solver)

            # --- Prepare temporal neural network input ---
            if rtnmpc.use_mlp and "temporal" in rtnmpc.mlp_metadata["MLPConfig"]["model_name"]:
                set_temporal_states_as_params(rtnmpc, ocp_solver, history, u_cmd)

            # --- Set parameters in OCP solver ---
            for j in range(ocp_solver.N + 1):
                ocp_solver.set(j, "p", rtnmpc.acados_parameters[j, :])

            ############################################################################################
            # --- Optimize control input ---
            # Compute control feedback and take the first action
            comp_time = time.time()
            try:
                # acados wrapper to solve the OCP and get first control command from sequence
                u_cmd = ocp_solver.solve_for_x0(state_curr)
            except Exception as e:
                print(f"Round {i}: acados ocp_solver returned status {ocp_solver.status}. Exiting.")
                # TODO try to recover from this gracefully
                raise e
            comp_time = (time.time() - comp_time) * 1000  # in ms

            # --- Sanity check constraints ---
            check_input_constraints(rtnmpc, u_cmd, i)
            ############################################################################################

            # --- Running history for temporal neural networks ---
            if rtnmpc.use_mlp and "temporal" in rtnmpc.mlp_metadata["MLPConfig"]["model_name"]:
                # Append current state and control to history for next iteration
                # Sorted from newest to oldest
                history = history[:-1, :]
                history = np.append(np.append(state_curr, u_cmd)[np.newaxis, :], history, axis=0)

            # --- Record time, target, current state and last optimized input ---
            if recording or plot:
                rec_dict["timestamp"] = np.append(rec_dict["timestamp"], t_now)
                rec_dict["dt"] = np.append(  # -2 since current timestamp is just added on index -1
                    rec_dict["dt"], t_now - rec_dict["timestamp"][-2] if len(rec_dict["timestamp"]) > 1 else T_samp
                )
                rec_dict["comp_time"] = np.append(rec_dict["comp_time"], comp_time)
                rec_dict["target"] = np.append(rec_dict["target"], state_ref[0:1, :], axis=0)
                rec_dict["state_in"] = np.append(rec_dict["state_in"], state_curr[np.newaxis, :], axis=0)
                rec_dict["control"] = np.append(rec_dict["control"], u_cmd[np.newaxis, :], axis=0)

            # --- Plot realtime ---
            if run_options["real_time_plot"]:
                # Note: Simulation is without disturbance here !
                #########################################
                # TODO OVERTHINK THIS!!!
                state_traj = simulate_trajectory(ocp_solver, sim_solver, state_curr)
                #########################################
                draw_robot(
                    art_pack,
                    None,  # TODO also display reference trajectory
                    None,
                    state_curr,
                    state_traj,
                    trajectory_history,
                    rotor_positions,
                    follow_robot=False,
                    animation=run_options["save_animation"],
                )

            # --- Simulate forward ---
            # Simulate with the optimized input until the next time step of the control period is reached
            # Note: Pretend to run simulation in parallel to the control loop
            # i.e., the control loop has no effect on the simulation loop execution times
            # Simply trigger new control optimization after simulating for T_samp seconds
            simulation_time = 0.0
            j = 0
            state_curr_sim = state_curr.copy()
            while simulation_time < T_samp:
                # Simulation runtime (inner loop)
                simulation_time += T_sim
                # --- Increment virtual time ---
                # ASSUMPTION: Simulation time is exactly equal to real time
                # i.e., the simulation has a zero runtime
                # This is somewhat realistic since in the real machine
                # the simulation (i.e. measurement + estimation) is run
                # in parallel to the real-time control loop.
                # Increment global time at every simulation step since the
                # control loop runs in parallel and is assumpted to be idle at some times
                t_now += T_sim

                # --- Set disturbance forces as parameters ---
                # TODO only apply disturbance to simulation model for next time step
                if rtnmpc.include_cog_dist_parameter:
                    apply_cog_disturbance(rtnmpc, sim_options["disturbances"]["cog_dist_factor"], u_cmd, state_curr)
                    if plot:
                        dist_dict["timestamp"] = np.append(dist_dict["timestamp"], t_now)
                        dist_dict["z"] = np.append(dist_dict["z"], state_curr_sim[2])
                        dist_dict["cog_dist"] = np.append(
                            dist_dict["cog_dist"],
                            rtnmpc.sim_acados_parameters[
                                np.newaxis, rtnmpc.cog_dist_start_idx : rtnmpc.cog_dist_end_idx
                            ],
                            axis=0,
                        )
                if rtnmpc.include_motor_noise_parameter:
                    apply_motor_noise(rtnmpc, u_cmd)
                    if plot:
                        if not rtnmpc.include_cog_dist_parameter:
                            dist_dict["timestamp"] = np.append(dist_dict["timestamp"], t_now)
                        dist_dict["motor_noise"] = np.append(
                            dist_dict["motor_noise"],
                            rtnmpc.sim_acados_parameters[
                                np.newaxis, rtnmpc.motor_noise_start_idx : rtnmpc.motor_noise_end_idx
                            ],
                            axis=0,
                        )

                # --- Simulate ---
                try:
                    if sim_options["use_real_world_simulator"] or sim_options["use_nominal_simulator"]:
                        state_curr_sim = sim_solver.simulate(
                            x=state_curr_sim, u=u_cmd, p=rtnmpc_sim.sim_acados_parameters
                        )
                    else:
                        state_curr_sim = sim_solver.simulate(x=state_curr_sim, u=u_cmd, p=rtnmpc.sim_acados_parameters)
                except Exception as e:
                    print(f"Round {i}.{j}: Error in simulation. Exiting.")
                    # TODO try to recover from this
                    raise e

                # --- Ensure unit quaternion ---
                state_curr_sim[6:10] = unit_quaternion(state_curr_sim[6:10])

                # --- Increment simulation step ---
                j += 1
            # --- Increment control step ---
            i += 1

            # --- Sanity check constraints ---
            check_state_constraints(ocp_solver, state_curr_sim, i)

            # --- Record out data ---
            if recording or plot:
                # State after simulation
                rec_dict["state_out"] = np.append(rec_dict["state_out"], state_curr_sim[np.newaxis, :], axis=0)

                ###################### OLD ###########################
                # if T_samp != T_step:
                #     raise ValueError("T_samp and T_step must be equal for prediction to make any sense since.")
                # NOTE this gets the state after T_step = 0.1 but the curr state is only passed for T_samp = 0.01
                # state_prop = ocp_solver.get(1, "x") # Predicted state by the controller at next sampling time
                ######################################################

                # Compute next state prediction through more precise and undisturbed integration
                state_prop = forward_prop(
                    dynamics_forward_prop,
                    state_forward_prop,
                    u_forward_prop,
                    state_curr[np.newaxis, :],
                    u_cmd[np.newaxis, :],
                    T_horizon=T_samp,
                    T_step=T_sim,  # T_prop_step,
                    num_stages=4,
                )
                state_prop = state_prop[-1, :]  # Get last predicted state
                rec_dict["state_prop"] = np.append(rec_dict["state_prop"], state_prop[np.newaxis, :], axis=0)

            # --- Log trajectory for real-time plot ---
            if run_options["real_time_plot"]:
                trajectory_history = np.append(trajectory_history, state_curr_sim[np.newaxis, :], axis=0)
                if len(trajectory_history) > 300:
                    trajectory_history = np.delete(trajectory_history, obj=0, axis=0)

            # Current target was reached!
            # --- Save data ---
            if recording and (i % 100 == 0 or t_now >= sim_options["max_sim_time"] or finished):
                write_recording_data(rec_dict, rec_file)
                rec_dict = make_blank_dict(nx, nx, nu)

            # --- Break condition for the outer loop ---
            if t_now >= sim_options["max_sim_time"]:
                break

    # --- End of simulation ---
    global_comp_time = time.time() - global_comp_time
    print(f"Trajectory tracking finished in {global_comp_time:.2f} seconds!")
    if recording:
        print(f"Recording finished. Data saved to {rec_file}")

    # --- Create video ---
    if run_options["save_animation"]:
        print("-------------- Saving animation as video --------------")
        dir_path = os.path.dirname(os.path.abspath(__file__))
        counter = 1
        while True:
            file_name = f"video/robot_animation_{str(counter).zfill(3)}.mp4"
            file_path = os.path.join(dir_path, file_name)
            if not os.path.exists(file_path):
                break
            counter += 1
        animate_robot(file_path)
        print(f"Saved in directory: {file_path}")

    # --- Plot simple trajectory ---
    if plot and not recording:
        plot_trajectory(model_options, rec_dict, rtnmpc, dist_dict=dist_dict, save=run_options["save_figures"])
        if rtnmpc.include_cog_dist_parameter or rtnmpc.include_motor_noise_parameter:
            plot_disturbances(dist_dict, save=run_options["save_figures"])
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
