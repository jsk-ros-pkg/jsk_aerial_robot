import time
import numpy as np
import matplotlib.pyplot as plt

from sim_environment.sim_solver import create_acados_sim_solver
from sim_environment.forward_prop import init_forward_prop, forward_prop
from utils.controller_utils import check_state_constraints, check_input_constraints
from utils.reference_utils import sample_random_target
from utils.geometry_utils import unit_quaternion, euclidean_dist
from visualize_comparison_icra_2026 import plot_comparison
from config.configurations import EnvConfig
from neural_controller_standalone import NeuralNMPC


def main(model_options, solver_options, dataset_options, sim_options, run_options):
    """
    IDEA: Control with neural model, simulate with neural model (trained on real data), record data for training
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
    model_options["only_use_nominal"] = False
    # Simulator (for neural controller)
    model_options["neural_model_instance"] = "neuralmodel_058"
    rtnmpc_neural_sim = NeuralNMPC(
        model_options=model_options,
        solver_options=solver_options,
        sim_options=sim_options,
        run_options=run_options,
        use_as_simulator=True,
    )
    sim_model_neural = rtnmpc_neural_sim.get_acados_model()
    sim_solver_neural = create_acados_sim_solver(rtnmpc_neural_sim, sim_model_neural, T_sim)

    # Simulator (for nominal controller) [NOTE: The simulator is still using the neural model, just like above]
    # rtnmpc_nominal_sim = NeuralNMPC(
    #     model_options=model_options, solver_options=solver_options, sim_options=sim_options, run_options=run_options,
    #     use_as_simulator=True
    # )
    # sim_solver_nominal = create_acados_sim_solver(rtnmpc_nominal_sim, T_sim)

    # Controller trained on nominal controller in neural simulator (58)
    model_options["neural_model_instance"] = "neuralmodel_060"
    solver_options["include_floor_bounds"] = True
    rtnmpc_neural_ctrl = NeuralNMPC(
        model_options=model_options, solver_options=solver_options, sim_options=sim_options, run_options=run_options
    )
    ocp_solver_neural_ctrl = rtnmpc_neural_ctrl.get_ocp_solver()
    ocp_model_neural_ctrl = rtnmpc_neural_ctrl.get_acados_model()
    reference_generator = rtnmpc_neural_ctrl.get_reference_generator()

    # Nominal controller (for comparison)
    model_options["only_use_nominal"] = True
    rtnmpc_nominal = NeuralNMPC(
        model_options=model_options, solver_options=solver_options, sim_options=sim_options, run_options=run_options
    )
    ocp_solver_nominal = rtnmpc_nominal.get_ocp_solver()

    # Recover some necessary variables from the NMPC object
    nx = ocp_model_neural_ctrl.x.shape[0]
    nu = ocp_model_neural_ctrl.u.shape[0]
    N = rtnmpc_neural_ctrl.N
    T_horizon = rtnmpc_neural_ctrl.T_horizon
    T_samp = rtnmpc_neural_ctrl.T_samp  # Time step for the control loop
    T_step = rtnmpc_neural_ctrl.T_step  # Time step in NMPC (= T_horizon / N)

    # Undisturbed model for creating labels to train on
    dynamics_forward_prop, state_forward_prop, u_forward_prop = init_forward_prop(rtnmpc_neural_ctrl)

    # --- Set initial state ---
    if run_options["initial_state"] is None:
        # state = [p, v, q, w, (a and/or t and/or ds)]
        state_curr_neural = np.zeros(nx)
        state_curr_neural[6] = 1.0  # Real part of quaternion
    else:
        state_curr_neural = run_options["initial_state"]
    state_curr_nominal = state_curr_neural.copy()
    state_curr_sim_neural = state_curr_neural.copy()
    state_curr_sim_nominal = state_curr_neural.copy()
    state_prop = state_curr_neural.copy()  # For forward prop prediction

    # --- Set target states ---
    if run_options["preset_targets"] is not None:
        targets = run_options["preset_targets"]
    else:
        targets = sample_random_target(
            np.array(state_curr_neural[:3]),
            sim_options["world_radius"],
            aggressive=run_options["aggressive"],
            low_flight=run_options["low_flight_targets"],
        )
    targets_reached = np.array([False for _ in targets])

    plot = run_options["plot_trajectory"]
    if plot:
        target_dim = targets.shape[1]
        state_dim = nx
        control_dim = nu
        rec_dict = {
            "timestamp": np.zeros((0, 1)),
            "dt": np.zeros((0, 1)),
            "comp_time_neural": np.zeros((0, 1)),
            "comp_time_nominal": np.zeros((0, 1)),
            "target": np.zeros((0, target_dim)),
            "state_in_neural": np.zeros((0, state_dim)),
            "state_in_nominal": np.zeros((0, state_dim)),
            "state_out_neural": np.zeros((0, state_dim)),
            "state_out_nominal": np.zeros((0, state_dim)),
            "state_prop": np.zeros((0, state_dim)),
            "control_neural": np.zeros((0, control_dim)),
            "control_nominal": np.zeros((0, control_dim)),
        }

    # --- Set up simulation ---
    u_cmd_neural_ctrl = None
    u_cmd_nominal = None
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
        rtnmpc_neural_ctrl.track(ocp_solver_neural_ctrl, state_ref, control_ref)
        rtnmpc_nominal.track(ocp_solver_nominal, state_ref, control_ref)

        # --------- NMPC loop ---------
        global_comp_time = time.time()
        while not current_target_reached:
            # --- Emergency recovery --- (quad controller gone out of control lol)
            if i > 1000:
                print("===== Emergency recovery triggered!!! =====")
                print(f"Iteration: {i}")
                print(f"Euclidean dist: {(current_target[:3] - state_curr_sim_neural[:3]) ** 2}")
                print(f"Current state: {state_curr_sim_neural}")
                i = 0
                ocp_solver_neural_ctrl.set(0, "x", state_ref[-1, :])
                ocp_solver_nominal.set(0, "x", state_ref[-1, :])
                sim_solver_neural.set("x", state_ref[-1, :])
                u_cmd_neural_ctrl = None
                u_cmd_nominal = None
                state_curr_neural = state_ref[-1, :]
                state_curr_nominal = state_ref[-1, :]

            # # --- Get current state ---
            if u_cmd_neural_ctrl is None:
                # If no command is available, use initial/last state
                sim_solver_neural.set("x", state_ref[-1, :])
            else:
                state_curr_neural = state_curr_sim_neural.copy()
                state_curr_nominal = state_curr_sim_nominal.copy()
                check_state_constraints(ocp_solver_neural_ctrl, state_curr_neural, i)

            # --- Set parameters in OCP solver ---
            for j in range(ocp_solver_neural_ctrl.N + 1):
                ocp_solver_neural_ctrl.set(j, "p", rtnmpc_neural_ctrl.acados_parameters[j, :])
            for j in range(ocp_solver_nominal.N + 1):
                ocp_solver_nominal.set(j, "p", rtnmpc_nominal.acados_parameters[j, :])

            ############################################################################################
            # --- Optimize control input ---
            # Compute control feedback and take the first action
            comp_time_neural = time.time()
            u_cmd_neural_ctrl = ocp_solver_neural_ctrl.solve_for_x0(state_curr_neural)
            comp_time_neural = (time.time() - comp_time_neural) * 1000  # in ms

            comp_time_nominal = time.time()
            u_cmd_nominal = ocp_solver_nominal.solve_for_x0(state_curr_nominal)
            comp_time_nominal = (time.time() - comp_time_nominal) * 1000  # in ms

            # --- Sanity check constraints ---
            check_input_constraints(rtnmpc_neural_ctrl, u_cmd_neural_ctrl, i)
            check_input_constraints(rtnmpc_nominal, u_cmd_nominal, i)
            ############################################################################################

            # --- Record time, target, current state and last optimized input ---
            if plot:  # or recording:
                rec_dict["timestamp"] = np.append(rec_dict["timestamp"], t_now)
                rec_dict["dt"] = np.append(  # -2 since current timestamp is just added on index -1
                    rec_dict["dt"], t_now - rec_dict["timestamp"][-2] if len(rec_dict["timestamp"]) > 1 else T_samp
                )
                rec_dict["comp_time_neural"] = np.append(rec_dict["comp_time_neural"], comp_time_neural)
                rec_dict["comp_time_nominal"] = np.append(rec_dict["comp_time_nominal"], comp_time_nominal)
                rec_dict["target"] = np.append(rec_dict["target"], current_target[np.newaxis, :], axis=0)
                rec_dict["state_in_neural"] = np.append(
                    rec_dict["state_in_neural"], state_curr_neural[np.newaxis, :], axis=0
                )
                rec_dict["state_in_nominal"] = np.append(
                    rec_dict["state_in_nominal"], state_curr_nominal[np.newaxis, :], axis=0
                )
                rec_dict["control_neural"] = np.append(
                    rec_dict["control_neural"], u_cmd_neural_ctrl[np.newaxis, :], axis=0
                )
                rec_dict["control_nominal"] = np.append(
                    rec_dict["control_nominal"], u_cmd_nominal[np.newaxis, :], axis=0
                )

            # --- Simulate forward ---
            # Simulate with the optimized input until the next time step of the control period is reached
            # Note: Pretend to run simulation in parallel to the control loop
            # i.e., the control loop has no effect on the simulation loop execution times
            # Simply trigger new control optimization after simulating for T_samp seconds
            simulation_time = 0.0
            j = 0
            state_curr_sim_neural = state_curr_neural.copy()
            state_curr_sim_nominal = state_curr_nominal.copy()
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
                state_curr_sim_neural = sim_solver_neural.simulate(
                    x=state_curr_sim_neural, u=u_cmd_neural_ctrl, p=rtnmpc_neural_sim.sim_acados_parameters
                )
                state_curr_sim_nominal = sim_solver_neural.simulate(
                    x=state_curr_sim_nominal, u=u_cmd_nominal, p=rtnmpc_neural_sim.sim_acados_parameters
                )

                # Ensure unit quaternion
                state_curr_sim_neural[6:10] = unit_quaternion(state_curr_sim_neural[6:10])
                state_curr_sim_nominal[6:10] = unit_quaternion(state_curr_sim_nominal[6:10])

                # Target check
                if euclidean_dist(current_target[:3], state_curr_sim_neural[:3], thresh=0.075):
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

                # Generate new target
                if run_options["preset_targets"] is None:
                    new_target = sample_random_target(
                        state_curr_sim_neural[:3],
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
                rec_dict["state_out_neural"] = np.append(
                    rec_dict["state_out_neural"], state_curr_sim_neural[np.newaxis, :], axis=0
                )
                rec_dict["state_out_nominal"] = np.append(
                    rec_dict["state_out_nominal"], state_curr_sim_nominal[np.newaxis, :], axis=0
                )

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
                    state_curr_neural[np.newaxis, :],
                    u_cmd_neural_ctrl[np.newaxis, :],
                    T_horizon=T_samp,
                    T_step=T_sim,  # T_prop_step,
                    num_stages=4,
                )
                state_prop = state_prop[-1, :]  # Get last predicted state
                rec_dict["state_prop"] = np.append(rec_dict["state_prop"], state_prop[np.newaxis, :], axis=0)

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

    # --- Plot simple trajectory ---
    if plot:  # and not recording:
        plot_comparison(rec_dict, rtnmpc_neural_ctrl)
        plt.show()


if __name__ == "__main__":
    main(
        EnvConfig.model_options,
        EnvConfig.solver_options,
        EnvConfig.dataset_options,
        EnvConfig.sim_options,
        EnvConfig.run_options,
    )
