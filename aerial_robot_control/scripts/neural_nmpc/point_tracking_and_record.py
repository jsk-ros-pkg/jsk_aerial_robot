import sys, os
import time
import argparse
import numpy as np

from utils.data_utils import get_recording_dict_and_file, make_blank_dict, write_recording_data
from utils.reference_utils import sample_random_target
from utils.geometry_utils import euclidean_dist
from utils.visualization_utils import initialize_plotter, draw_robot, animate_robot
from config.configurations import EnvConfig
from neural_controller import NeuralNMPC


np.random.seed(123)  # Set seed for reproducibility

def main(model_options, solver_options, dataset_options, sim_options, run_options):
    """
    Main function to run the NMPC simulation and recording.
    :param model_options: Options for the NMPC model.
    :param dataset_options: Options for the recording.
    :param sim_options: Options for the simulation.
    :param run_options: Additional parameters for the simulation.
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
    rtnmpc = NeuralNMPC(model_options=model_options, solver_options=solver_options,
                        sim_options=sim_options, run_options=run_options)

    ocp_solver = rtnmpc.ocp_solver
    # Create sim solver for the extended model
    sim_solver = rtnmpc.create_acados_sim_solver(T_sim)

    reference_generator = rtnmpc.get_reference_generator()

    # Recover some necessary variables from the NMPC object
    nx = rtnmpc.acados_model.x.shape[0]
    nu = rtnmpc.acados_model.u.shape[0]
    N = rtnmpc.N
    T_horizon = rtnmpc.T_horizon
    T_samp = rtnmpc.T_samp  # Time step for the control loop
    T_step = rtnmpc.T_step  # Time step in NMPC (= T_horizon / N)
    # reference_over_sampling = 1     # TODO what is this?
    # control_period = T_horizon / (N * reference_over_sampling)    # The time period between two control inputs

    # Sanity check: The optimization should be faster or equal than the duration of the optimization time step
    assert T_samp <= T_horizon / N
    assert T_samp >= T_sim

    # --- Set initial state ---
    if run_options["initial_state"] is None:
        # state = [p, v, q, w, (a and or t and or ds)]
        state_curr = np.zeros(nx)
        state_curr[6] = 1.0     # Real quaternion
    else:
        state_curr = run_options["initial_state"]
    state_curr_sim = state_curr.copy()

    # --- Set target states ---
    if run_options["preset_targets"] is not None:
        targets = run_options["preset_targets"]
    else:
        targets = sample_random_target(np.array(state_curr[:3]), sim_options["world_radius"],
                                       aggressive=run_options["aggressive"])
    targets_reached = np.array([False for _ in targets])

    # --- Prepare recording ---
    recording = run_options["recording"]
    if recording:
        # Create an empty dict or get a pre-recorded dict and filepath to store
        # TODO actually able to use a pre-recorded dict? And if so make it overwriteable
        model_options["state_dim"] = nx
        model_options["control_dim"] = nu
        model_options["nmpc_params"] = rtnmpc.nmpc.params
        ds_name = model_options["nmpc_type"] + "_" + dataset_options["ds_name_suffix"]
        rec_dict, rec_file = get_recording_dict_and_file(ds_name, model_options, sim_options, solver_options, targets[0].size)

        if run_options["real_time_plot"]:
            run_options["real_time_plot"] = False
            print("Turned off real time plot during recording mode.")

    # --- Real time plot ---
    # Generate necessary art pack for real time plot
    if run_options["real_time_plot"]:
        art_pack = initialize_plotter(world_rad=sim_options["world_radius"], n_properties=N)
        trajectory_history = state_curr[np.newaxis, :]
        rotor_positions = rtnmpc.get_rotor_positions()

    plot = run_options["plot_traj"]
    if plot:
        rec_dict = make_blank_dict(targets[0].size, nx, nu)

    # --- Set up simulation ---
    u_cmd = None
    i = 0; j = 0
    t_now = 0.0  # Total virtual time in seconds

    # ---------- Targets loop ----------
    print("Targets reached: ", end='')
    while False in targets_reached:
        # --- Target ---
        current_target_idx = np.where(targets_reached == False)[0][0]
        current_target = targets[current_target_idx]
        current_target_reached = False

        # --- Reference ---
        # Compute reference for Input u with an allocation matrix - TODO still makes sense if we don't know model in the first place?
        # Alternative is setting the modular trajectory yref dynamically in control loop
        state_ref, control_ref = reference_generator.compute_trajectory(target_xyz=current_target[:3], target_rpy=current_target[6:9])
        # Track reference in solver over horizon
        rtnmpc.track(state_ref, control_ref)

        # --------- NMPC loop ---------
        global_comp_time = time.time()
        while not current_target_reached:

            # --- Emergency recovery --- (quad controller gone out of control lol)
            if np.any(state_curr[7:10] > 14) or i > 500: # TODO why check quaternions to be > 14?
                print("===== Emergency recovery triggered!!! =====")
                print("Iteration: ", i, " | Current state: ", state_curr)
                i = 0
                ocp_solver.set(0, "x", state_ref[-1,:])
                sim_solver.set("x", state_ref[-1,:])

            # --- Get current state ---
            if u_cmd is not None:
                # Basically don't overwrite initial state
                state_curr = sim_solver.get("x")
                rtnmpc.check_state_constraints(state_curr, i)
            else:
                # Set initial state in solver
                sim_solver.set("x", state_curr)

            # --- Initial guess ---
            # TODO set initial guess to prev iteration?
            # TODO Provide a new initial guess when changing target
            # initial_guess = rtnmpc.reshape_input_sequence(u_cmd)
            # # TODO understand: "Save initial guess for future optimization. It is a time-shift of the current optimized variables"
            # initial_guess = np.array(cs.vertcat(initial_guess[1:, :], cs.DM.zeros(4).T))

            # --- Optimize control input ---
            # Compute control feedback and take the first action
            comp_time = time.time()
            try:
                u_cmd = ocp_solver.solve_for_x0(state_curr)  # Wrapper to solve the OCP and get first command
            except Exception as e:
                print(f"Round {i}: acados ocp_solver returned status {ocp_solver.status}. Exiting.")
                # TODO try to recover from this
                continue # raise e
            comp_time = (time.time() - comp_time) * 1000  # in ms

            # --- Sanity check constraints ---
            rtnmpc.check_input_constraints(u_cmd, i)

            # --- Record time, target, current state and last optimized input ---
            if recording or plot:
                rec_dict["timestamp"] = np.append(rec_dict["timestamp"], t_now)
                rec_dict["dt"] = np.append(rec_dict["dt"], t_now - rec_dict["timestamp"][-2] if len(rec_dict["timestamp"]) > 1 else T_samp)
                rec_dict["comp_time"] = np.append(rec_dict["comp_time"], comp_time)
                rec_dict["target"] = np.append(rec_dict["target"], current_target[np.newaxis, :], axis=0)
                rec_dict["state_in"] = np.append(rec_dict["state_in"], state_curr[np.newaxis, :], axis=0)
                rec_dict["control"] = np.append(rec_dict["control"], u_cmd[np.newaxis, :], axis=0)

            # if recording:
            #     # Integrate first input. Will be used as nominal model prediction during next save
            #     state_pred, _ = rtnmpc.forward_prop(np.squeeze(state_curr), u_cmd=u_cmd[:4],
            #                                       T_horizon=control_period, use_gp=False)
            #     state_pred = state_pred[-1, :]

            # --- Plot realtime ---
            if run_options["real_time_plot"]:
                trajectory_pred = rtnmpc.simulate_trajectory(sim_solver)
                draw_robot(art_pack, targets, targets_reached, state_curr,
                           trajectory_pred, trajectory_history,
                           rotor_positions, follow_robot=False, 
                           animation=run_options["save_animation"])

            # --- Simulate forward ---
            # Simulate with the optimized input until the next time step of the control period is reached
            # Note: Pretend to run simulation in parallel to the control loop
            # i.e., the control loop has no effect on the simulation loop execution times
            # Simply trigger new control optimization after simulating for T_samp seconds
            simulation_time = 0.0
            j = 0
            state_curr_sim = state_curr.copy()  # TODO kind of redundant
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

                # Simulate
                try:
                    state_curr_sim = sim_solver.simulate(x=state_curr_sim, u=u_cmd)
                except Exception as e:
                    print(f"Round {i}.{j}: acados ocp_solver returned status {sim_solver.status}. Exiting.")
                    # TODO try to recover from this
                    raise e

                # Target check
                if euclidean_dist(current_target[:3], state_curr_sim[:3], thresh=0.05):
                    # Target reached!
                    print("*", end='')
                    sys.stdout.flush()

                    # Mark current target as reached
                    current_target_reached = True
                    targets_reached[current_target_idx] = True
                    i = 0; j = 0
                    simulation_time = 0.0

                    # # Remove initial guess
                    # initial_guess = None

                    # Generate new target
                    if run_options["preset_targets"] is None:
                        new_target = sample_random_target(state_curr_sim[:3], sim_options["world_radius"],
                                                          aggressive=run_options["aggressive"])
                        targets = np.append(targets, new_target, axis=0)
                        targets_reached = np.append(targets_reached, False)
                    break
                # --- Increment simulation step ---
                j += 1
            # --- Increment control step ---
            if not current_target_reached:
                i += 1

            # --- Record out data ---
            if recording or plot:
                # TODO this gets the state after T_step = 0.1 but the curr state is only passed for T_samp = 0.01
                if T_samp != T_step:
                    raise ValueError("T_samp and T_step must be equal for prediction to make any sense since.")
                state_pred = ocp_solver.get(1, "x") # Predicted state by the controller at next sampling time
                rec_dict["state_out"] = np.append(rec_dict["state_out"], state_curr_sim[np.newaxis, :], axis=0)
                rec_dict["state_pred"] = np.append(rec_dict["state_pred"], state_pred[np.newaxis, :], axis=0)

                error = state_curr_sim - state_pred
                rec_dict["error"] = np.append(rec_dict["error"], error[np.newaxis, :], axis=0)

            # --- Log trajectory for real-time plot ---
            if run_options["real_time_plot"]:
                trajectory_history = np.append(trajectory_history, state_curr_sim[np.newaxis, :], axis=0)
                if len(trajectory_history) > 300:
                    trajectory_history = np.delete(trajectory_history, obj=0, axis=0) # Delete first logged state
        
            # --- Break condition for the inner loop ---
            if t_now >= sim_options["max_sim_time"]:
                break

        # Current target was reached!
        # --- Save data ---
        if recording:
            write_recording_data(rec_dict, rec_file)
            rec_dict = make_blank_dict(targets[0].size, nx, nu)

        # --- Break condition for the outer loop ---
        if t_now >= sim_options["max_sim_time"]:
            break

        print(f"Computation time for target {current_target_idx}: {time.time() - global_comp_time}")


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
    if plot:
        plot_trajectory(rec_dict, rtnmpc)

def plot_trajectory(rec_dict, rtnmpc):
    import matplotlib.pyplot as plt
    fig = plt.subplots(figsize=(20, 5))
    # Plot input features
    state_in = rec_dict["state_in"]
    state_out = rec_dict["state_out"]
    state_pred = rec_dict["state_pred"]
    # x = np.concatenate(state_in, rec_dict["control"])
    n_plots = state_in.shape[1]#, x.shape[1])
    for dim in range(state_in.shape[1]):
        plt.subplot(n_plots, 2, dim * 2 + 1)
        plt.plot(state_in[:, dim], label='state_in')
        plt.plot(state_out[:, dim], label='state_out')
        plt.plot(state_pred[:, dim], label='state_pred')
        plt.ylabel(f'D{dim}')
        if dim == 0:
            plt.title('State In & State Out')
            plt.legend()
        plt.grid('on')

    control = rec_dict["control"]
    for dim in range(control.shape[1]):
        plt.subplot(n_plots, 2, dim * 2 + 2)
        plt.plot(control[:, dim], label='control')
        if dim == 0:
            plt.title('Control')
            plt.legend()
        plt.grid('on')

    plt.tight_layout()
    plt.show()

    fig = plt.figure(figsize=(20, 5))
    plt.plot(rec_dict["comp_time"])
    plt.plot([0, rec_dict["comp_time"].shape[0]], [np.mean(rec_dict['comp_time']), np.mean(rec_dict['comp_time'])], color='r', label=f"Avg = {np.mean(rec_dict['comp_time']):.4f} ms")
    plt.xlabel('Simulation time [s]')
    plt.ylabel('Computation time [ms]')
    plt.legend()
    plt.grid()
    plt.tight_layout()
    plt.show()

    dt = np.expand_dims(rec_dict["dt"], 1)

    diff1 = state_out - state_in
    d1 = diff1 / dt
    diff2 = state_out - state_pred

    fig = plt.subplots(figsize=(20, 5))
    for dim in range(state_in.shape[1]):
        plt.subplot(n_plots, 2, dim * 2 + 1)
        plt.plot(d1[:, dim], label='d1', color='red')
        plt.ylabel(f'D{dim}')
        if dim == 0:
            plt.title('State Out - State In')
            plt.legend()
        plt.grid('on')

    for dim in range(state_in.shape[1]):
        plt.subplot(n_plots, 2, dim * 2 + 2)
        plt.plot(diff2[:, dim], label='diff2', color='green')
        if dim == 0:
            plt.title('State Out - State Pred')
            plt.legend()
        plt.grid('on')

    plt.tight_layout()
    plt.show()

    plt.subplots(figsize=(10, 5))
    import torch
    y = torch.zeros((state_in.shape[0], rtnmpc.y_reg_dims.shape[0])).type(torch.float32).to(torch.device("cuda"))
    for t in range(state_in.shape[0]):
        s = torch.from_numpy(state_in[t, rtnmpc.state_feats]).type(torch.float32).to(torch.device("cuda"))
        u = torch.from_numpy(control[t, rtnmpc.u_feats]).type(torch.float32).to(torch.device("cuda"))
        x = torch.cat((s, u)).unsqueeze(0)  # Add batch dimension
        rtnmpc.neural_model.eval()
        y[t] = rtnmpc.neural_model(x)

    y = y.cpu().detach().numpy()
    y_true = d1 # state_out - state_pred
    for dim in range(y.shape[1]):
        plt.subplot(y.shape[1], 1, dim+1)
        plt.plot(y[:, dim], label='y_regressed')
        plt.plot(y[:,dim] - y_true[:, dim], label='error', color='r', linestyle='--', alpha=0.5)
        plt.plot(y_true[:, dim], label='y_true', color="orange")
        # plt.plot(state_out[:, dim+3], label='state_out', linestyle='--')
        # plt.plot(state_out[:, dim+3] - y[:, dim], label='error', linestyle=':')
        plt.ylabel(f'D{dim}')
        if dim == 0:
            plt.title('State Out - State In')
            plt.legend()
        plt.grid('on')
    plt.tight_layout()
    plt.show()

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
    """
if __name__ == '__main__':

    main(EnvConfig.model_options, EnvConfig.solver_options,
         EnvConfig.dataset_options, EnvConfig.sim_options,
         EnvConfig.run_options)
