import sys, os
import time
import numpy as np
import pandas as pd
import torch
import matplotlib.pyplot as plt
from config.configurations import DirectoryConfig, EnvConfig, ModelFitConfig
from utils.data_utils import undo_jsonify
from utils.geometry_utils import v_dot_q, quaternion_inverse
from utils.model_utils import load_model
from utils.statistics_utils import prune_dataset
from sim_environment.forward_prop import init_forward_prop
from neural_controller import NeuralNMPC

sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from nmpc.nmpc_tilt_mt.tilt_qd import phys_param_beetle_omni as phys_omni


class struct(object):
    pass


def main():
    ###################### SETUP ######################
    # Settings
    file_name = (
        # "/NMPCTiltQdServo_residual_dataset_04/dataset_001.csv"
        # "/NMPCTiltQdServo_real_machine_dataset_01/dataset_013.csv"
        "/NMPCTiltQdServo_real_machine_dataset_GROUND_EFFECT_ONLY/dataset_001.csv"
        # "/NMPCTiltQdServo_real_machine_dataset_VAL_FOR_PAPER/dataset_003.csv"
        # "/NMPCTiltQdServo_residual_dataset_neural_sim_nominal_control_07/dataset_001.csv"
        # VAL: 1 (base), 3 (with ref)
    )
    df = pd.read_csv(DirectoryConfig.DATA_DIR + file_name[0])
    vz_idx = 5
    q_idx = 6
    # state_feats = np.array([2, 3, 4, 5, 6, 7, 8, 9])
    state_feats = np.array([2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12])
    u_feats = np.array([0, 1, 2, 3, 4, 5, 6, 7])
    # y_reg_dims = np.array([5])
    y_reg_dims = np.array([3, 4, 5])

    # Define and load model
    model_options = EnvConfig.model_options
    solver_options = EnvConfig.solver_options
    sim_options = EnvConfig.sim_options
    run_options = EnvConfig.run_options
    neural_model, mlp_metadata = load_model(model_options, sim_options, run_options)
    neural_model.eval()

    # Data
    timestamp = df["timestamp"].to_numpy()
    dt = df["dt"].to_numpy()[:, np.newaxis]
    state_in = undo_jsonify(df["state_in"].to_numpy())
    state_out = undo_jsonify(df["state_out"].to_numpy())
    state_prop = undo_jsonify(df["state_prop"].to_numpy())
    control = undo_jsonify(df["control"].to_numpy())
    if "position_ref" in df.columns and "quaternion_ref" in df.columns:
        has_ref = True
        pos_ref = undo_jsonify(df["position_ref"].to_numpy())
        quat_ref = undo_jsonify(df["quaternion_ref"].to_numpy())
    elif "state_ref" in df.columns:
        has_ref = True
        state_ref = undo_jsonify(df["state_ref"].to_numpy())
        pos_ref = state_ref[:, :3]
        quat_ref = state_ref[:, 6:10]
    else:
        has_ref = False

    # Label
    diff = (state_out - state_prop) / dt
    diff_const = (state_out - state_prop) / 0.01

    # Pruning
    if ModelFitConfig.prune:
        # Remove outliers with histogram bins where the total ratio of data is lower than a threshold
        vel_idx = np.array([vz_idx - 2, vz_idx - 1, vz_idx])

        if set(np.array(vel_idx)).issubset(set(y_reg_dims)):
            y_acc_idx = np.where(np.in1d(y_reg_dims, vel_idx))[0]
        elif set([vz_idx]).issubset(set(y_reg_dims)):
            y_acc_idx = np.where(np.in1d(y_reg_dims, vz_idx))[0]

        x_vel = state_in[:, vel_idx]
        y_acc = diff[:, y_acc_idx]

        histogram_bins = ModelFitConfig.histogram_n_bins
        histogram_threshold = ModelFitConfig.histogram_thresh
        velocity_cap = ModelFitConfig.vel_cap  # Also remove dataset points if abs(velocity) > velocity_cap

        prune_idx = prune_dataset(x_vel, y_acc, velocity_cap, histogram_bins, histogram_threshold, plot=False)
        timestamp = timestamp[prune_idx]
        dt = dt[prune_idx, :]
        state_in = state_in[prune_idx, :]
        state_out = state_out[prune_idx, :]
        state_prop = state_prop[prune_idx, :]
        control = control[prune_idx, :]
        diff = diff[prune_idx, :]
        diff_const = diff_const[prune_idx, :]
        if has_ref:
            pos_ref = pos_ref[prune_idx, :]
            quat_ref = quat_ref[prune_idx, :]

    ###################### NETWORK INPUT/OUTPUT ######################
    # MLP input
    def velocity_mapping(state_sequence):
        p_traj = state_sequence[:, :3]
        v_w_traj = state_sequence[:, 3:6]
        q_traj = state_sequence[:, 6:10]
        other_traj = state_sequence[:, 10:]  # w, a_s, f_s, etc.

        v_b_traj = np.empty_like(v_w_traj)
        for t in range(len(v_w_traj)):
            v_b_traj[t, :] = v_dot_q(v_w_traj[t, :], quaternion_inverse(q_traj[t, :]))
        return np.concatenate((p_traj, v_b_traj, q_traj, other_traj), axis=1)

    if mlp_metadata["ModelFitConfig"]["input_transform"]:
        state_in_mlp = velocity_mapping(state_in)
    else:
        # Don't transform input but let network learn in world frame directly
        state_in_mlp = state_in.copy()

    if mlp_metadata["ModelFitConfig"]["label_transform"]:
        state_out_mlp = velocity_mapping(state_out)
        state_prop_mlp = velocity_mapping(state_prop)
    else:
        # Don't transform output but let network learn in world frame directly
        state_out_mlp = state_out.copy()
        state_prop_mlp = state_prop.copy()

    diff_mlp = (state_out_mlp - state_prop_mlp) / dt
    diff_const_mlp = (state_out_mlp - state_prop_mlp) / 0.01

    device = "cpu"  # "cuda"
    state_in_mlp_tensor = torch.from_numpy(state_in_mlp).type(torch.float32).to(torch.device(device))
    control_tensor = torch.from_numpy(control).type(torch.float32).to(torch.device(device))
    mlp_in = torch.cat((state_in_mlp_tensor[:, state_feats], control_tensor[:, u_feats]), axis=1)

    plt.figure(figsize=(20, 5))
    plt.subplot(2, 1, 1)
    plt.title("State In, Out & Prop MLP (poss. transformed)")
    plt.plot(timestamp, state_in_mlp[:, 2], label="state_in_mlp")
    plt.plot(timestamp, state_out_mlp[:, vz_idx], label="state_out_mlp")
    plt.plot(timestamp, state_prop_mlp[:, vz_idx], label="state_prop_mlp")
    plt.xlim(timestamp[0], timestamp[-1])
    plt.ylabel("vz")
    plt.legend()
    plt.grid("on")
    plt.subplot(2, 1, 2)
    plt.title("Labels [(state_out_mlp - state_prop_mlp) / dt]")
    plt.plot(timestamp, diff_mlp[:, vz_idx], color="green")
    plt.xlim(timestamp[0], timestamp[-1])
    plt.ylabel("az")
    plt.grid("on")

    # MLP output
    mlp_out = neural_model(mlp_in).detach().cpu().numpy()

    # Unpack prediction outputs. Transform back to world reference frame
    if mlp_metadata["ModelFitConfig"]["label_transform"]:
        if (y_reg_dims != np.array([3, 4, 5])).all():
            raise NotImplementedError("Only implemented for ax, ay, az output.")
        for t in range(mlp_out.shape[0]):
            mlp_out[t, :] = v_dot_q(mlp_out[t, :], state_in_mlp[t, q_idx : q_idx + 4])

    # Scale by weight
    if model_options["scale_by_weight"]:
        mlp_out /= nmpc.phys.mass

    # Plot true labels vs. actual regression
    y = mlp_out
    y_true = diff_mlp
    y_true_const = diff_const_mlp
    plt.subplots(figsize=(10, 5))
    for i, dim in enumerate(y_reg_dims):
        plt.subplot(y.shape[1], 1, i + 1)
        plt.plot(timestamp, y[:, i] - y_true[:, dim], label="error", color="r", linestyle="--", alpha=0.5)
        plt.plot(timestamp, y_true[:, dim], label="y_true", color="orange")
        plt.plot(timestamp, y_true_const[:, dim], label="y_true_const", color="tab:brown", alpha=0.5)
        plt.plot(timestamp, y[:, i], label="y_regressed")
        plt.xlim(timestamp[0], timestamp[-1])
        plt.ylabel(f"D{dim}")
        plt.grid("on")
        if i == 0:
            plt.title("Model output vs. True label")
            plt.legend()
        if i != y.shape[1] - 1:
            ax = plt.gca()
            ax.axes.xaxis.set_ticklabels([])

    # Plot only model output
    plt.subplots(figsize=(10, 5))
    plt.title("Model output")
    for i, dim in enumerate(y_reg_dims):
        plt.subplot(y.shape[1], 1, i + 1)
        plt.plot(timestamp, y[:, i], label="y_regressed")
        plt.xlim(timestamp[0], timestamp[-1])
        plt.ylabel(f"D{dim}")
        plt.legend()
        plt.grid("on")
        if i != y.shape[1] - 1:
            ax = plt.gca()
            ax.axes.xaxis.set_ticklabels([])

    # Plot loss per dimension
    loss = np.square(y_true[:, y_reg_dims] - y)
    loss_const = np.square(y_true_const[:, y_reg_dims] - y)
    plt.subplots(figsize=(10, 5))
    for i, dim in enumerate(y_reg_dims):
        if dim == 0:
            plt.title("Network Loss per Dimension")
        plt.subplot(y.shape[1], 1, i + 1)
        plt.plot(timestamp, loss_const[:, i], color="tab:blue")
        plt.plot(timestamp, loss[:, i], color="tab:cyan")
        plt.plot(
            [0, loss.shape[0]],
            [np.mean(loss[:, i]), np.mean(loss[:, i])],
            color="blue",
            linestyle="--",
            label=f"Mean = {np.mean(loss[:, i]):.6f}",
        )
        plt.plot(
            [0, loss_const.shape[0]],
            [np.mean(loss_const[:, i]), np.mean(loss_const[:, i])],
            color="tab:green",
            linestyle="--",
            label=f"Mean (const. dt) = {np.mean(loss_const[:, i]):.6f}",
        )
        plt.xlim(timestamp[0], timestamp[-1])
        plt.ylabel(f"Loss D{dim}")
        plt.legend()
        plt.grid("on")
        if i != y.shape[1] - 1:
            ax = plt.gca()
            ax.axes.xaxis.set_ticklabels([])

    # Plot total loss and RMSE
    plt.figure(figsize=(10, 5))
    total_loss = np.sum(loss, axis=1)
    rmse = np.sqrt(total_loss / y.shape[1])
    plt.plot(timestamp, total_loss, label="Total Loss", color="red")
    plt.plot(timestamp, rmse, label="RMSE", color="green")
    plt.plot(
        [0, total_loss.shape[0]],
        [np.mean(total_loss), np.mean(total_loss)],
        color="red",
        linestyle="--",
        alpha=0.7,
        label=f"Mean Total Loss = {np.mean(total_loss):.6f}",
    )
    plt.plot(
        [0, loss.shape[0]],
        [np.mean(rmse), np.mean(rmse)],
        color="green",
        linestyle="--",
        alpha=0.7,
        label=f"Mean RMSE = {np.mean(rmse):.6f}",
    )
    plt.xlim(timestamp[0], timestamp[-1])
    plt.xlabel("Time [s]")
    plt.ylabel("Loss / RMSE")
    plt.title("Neural Model Total Loss and RMSE")
    plt.legend()
    plt.grid("on")

    ###################### MODEL OUTPUT ######################
    # Simulate intermediate acceleration before integration
    nmpc = struct()
    nmpc.tilt = True
    nmpc.include_servo_model = True
    nmpc.include_thrust_model = False
    nmpc.include_servo_derivative = False
    nmpc.include_cog_dist_parameter = True
    nmpc.phys = phys_omni

    # Define nominal model
    dynamics, _, _ = init_forward_prop(nmpc)

    # Compute linear acceleration with nominal model
    x_dot = np.empty(state_in.shape)
    for t in range(state_in.shape[0]):
        x_dot[t, :] = np.array(dynamics(x=state_in[t, :], u=control[t, :])["x_dot"]).squeeze()
    lin_acc = x_dot[:, 3:6]

    # CoG disturbance (resimulate)
    if sim_options["disturbances"]["cog_dist"]:
        cog_dist = np.zeros((control.shape[0], 6))
        for t in range(control.shape[0]):
            u_cmd = control[t, :]
            max_thrust = np.average(u_cmd[:4])
            # Ground effect increases lift the closer drone is to the ground
            # Force values behave in [-thrust_max, thrust_max]
            z = state_in[t, 2]
            cog_dist_factor = sim_options["disturbances"]["cog_dist_factor"]
            force_mu_z = 1 / (z + 1) ** 2 * cog_dist_factor * max_thrust * 4

            force_std_z = 0
            force_mu_x = 0
            force_mu_y = 0
            force_std_x = 0
            force_std_y = 0
            torque_mu = 0
            torque_std = 0

            mu = np.array([force_mu_x, force_mu_y, force_mu_z, torque_mu, torque_mu, torque_mu])
            std = np.array([force_std_x, force_std_y, force_std_z, torque_std, torque_std, torque_std])
            cog_dist[t, :] = mu  # np.random.normal(loc=mu, scale=std)
        lin_acc_dist = lin_acc + cog_dist[:, :3] / nmpc.phys.mass

    # Plot simulation results for acceleration and the effect of the neural model
    if (y_reg_dims == np.array([5])).all():
        # Only vz
        plt.figure()
        plt.title("Model Output")
        plt.plot(timestamp, lin_acc[:, 2], label="Acceleration by undisturbed model", color="tab:blue")
        if sim_options["disturbances"]["cog_dist"]:
            plt.plot(timestamp, lin_acc_dist[:, 2], label="Acceleration by disturbed model", color="tab:olive")
            plt.plot(timestamp, lin_acc_dist[:, 2] + y[:, 0], label="Neural Compensation (+)", color="tab:orange")
            plt.plot(timestamp, lin_acc_dist[:, 2] - y[:, 0], label="Neural Compensation (-)", color="tab:brown")
        else:
            plt.plot(timestamp, lin_acc[:, 2] + y[:, 0], label="Neural Compensation (+) (no dist)", color="tab:orange")
            plt.plot(timestamp, lin_acc[:, 2] - y[:, 0], label="Neural Compensation (-) (no dist)", color="tab:brown")
        plt.xlim(timestamp[0], timestamp[-1])
        plt.ylabel(f"D5")
        plt.legend()
        plt.grid("on")

        plt.figure()
        if sim_options["disturbances"]["cog_dist"]:
            plt.plot(timestamp, lin_acc_dist[:, 2] + y[:, 0], label="Neural Compensation (+)", color="tab:orange")
        else:
            plt.plot(timestamp, lin_acc[:, 2] + y[:, 0], label="Neural Compensation (+) (no dist)", color="tab:orange")
        plt.xlim(timestamp[0], timestamp[-1])
        plt.ylabel(f"D5")
        plt.legend()
        plt.grid("on")
    else:
        plt.subplots(figsize=(10, 5))
        for i, dim in enumerate(y_reg_dims):
            plt.subplot(len(y_reg_dims), 1, i + 1)
            plt.plot(timestamp, lin_acc[:, i], label="Acceleration by undisturbed model", color="tab:blue")
            if sim_options["disturbances"]["cog_dist"]:
                plt.plot(timestamp, lin_acc_dist[:, i], label="Acceleration by disturbed model", color="tab:olive")
                plt.plot(timestamp, lin_acc_dist[:, i] + y[:, i], label="Neural Compensation (+)", color="tab:orange")
                plt.plot(timestamp, lin_acc_dist[:, i] - y[:, i], label="Neural Compensation (-)", color="tab:brown")
            else:
                plt.plot(
                    timestamp, lin_acc[:, i] + y[:, i], label="Neural Compensation (+) (no dist)", color="tab:orange"
                )
                plt.plot(
                    timestamp, lin_acc[:, i] - y[:, i], label="Neural Compensation (-) (no dist)", color="tab:brown"
                )
            plt.xlim(timestamp[0], timestamp[-1])
            plt.ylabel(f"D{dim}")
            plt.grid("on")
            if i == 0:
                plt.title("Model Output")
                plt.legend()
            if i != y.shape[1] - 1:
                ax = plt.gca()
                ax.axes.xaxis.set_ticklabels([])

        plt.subplots(figsize=(10, 5))
        for i, dim in enumerate(y_reg_dims):
            plt.subplot(len(y_reg_dims), 1, i + 1)
            if sim_options["disturbances"]["cog_dist"]:
                plt.plot(timestamp, lin_acc_dist[:, i] - y[:, i], label="Neural Compensation (-)", color="tab:brown")
                plt.plot(timestamp, lin_acc_dist[:, i] + y[:, i], label="Neural Compensation (+)", color="tab:orange")
            else:
                plt.plot(
                    timestamp, lin_acc[:, i] - y[:, i], label="Neural Compensation (-) (no dist)", color="tab:brown"
                )
                plt.plot(
                    timestamp, lin_acc[:, i] + y[:, i], label="Neural Compensation (+) (no dist)", color="tab:orange"
                )
            plt.xlim(timestamp[0], timestamp[-1])
            plt.ylabel(f"D{dim}")
            plt.grid("on")
            if i == 0:
                plt.title("Model Output")
                plt.legend()
            if i != y.shape[1] - 1:
                ax = plt.gca()
                ax.axes.xaxis.set_ticklabels([])

    ###################### GENERAL ######################
    # Plot input state features
    plt.subplots(figsize=(20, 5))
    for dim in range(state_in.shape[1]):
        plt.subplot(state_in.shape[1], 2, dim * 2 + 1)
        plt.plot(timestamp, state_in[:, dim], label="state_in")
        plt.plot(timestamp, state_out[:, dim], label="state_out")
        plt.plot(timestamp, state_prop[:, dim], label="state_prop")
        plt.xlim(timestamp[0], timestamp[-1])
        plt.ylabel(f"Feature {dim}")
        plt.grid("on")
        if dim == 0:
            plt.title("State In & State Out")
            plt.legend()
        if dim != state_in.shape[1] - 1:
            ax = plt.gca()
            ax.axes.xaxis.set_ticklabels([])

    # Plot residual dynamics
    for dim in range(state_in.shape[1]):
        plt.subplot(state_in.shape[1], 2, dim * 2 + 2)
        plt.plot(timestamp, diff[:, dim], color="green")
        plt.xlim(timestamp[0], timestamp[-1])
        plt.ylabel(f"D{dim}")
        plt.grid("on")
        if dim == 0:
            plt.title("State Out - State Pred")
        if dim != state_in.shape[1] - 1:
            ax = plt.gca()
            ax.axes.xaxis.set_ticklabels([])

    # Plot absolute velocity and its residual dynamics in z
    plt.figure(figsize=(20, 5))
    plt.subplot(2, 1, 1)
    plt.title("Zoom in Dim vz")
    plt.plot(timestamp, state_in[:, vz_idx], label="state_in")
    plt.plot(timestamp, state_out[:, vz_idx], label="state_out")
    plt.plot(timestamp, state_prop[:, vz_idx], label="state_prop")
    plt.xlim(timestamp[0], timestamp[-1])
    plt.ylabel("vz")
    plt.legend()
    plt.grid("on")
    plt.subplot(2, 1, 2)
    plt.title("Label [(State Out - State Pred) / dt]")
    plt.plot(timestamp, diff[:, vz_idx], color="green")
    plt.xlim(timestamp[0], timestamp[-1])
    plt.ylabel("vz")
    plt.grid("on")

    plt.figure(figsize=(20, 5))
    plt.subplot(2, 1, 1)
    plt.title("Zoom in Dim vz (only State In)")
    plt.plot(timestamp, state_in[:, 2])
    plt.xlim(timestamp[0], timestamp[-1])
    plt.ylabel("z")
    plt.grid("on")
    plt.subplot(2, 1, 2)
    plt.title("Label [(State Out - State Pred) / dt]")
    plt.plot(timestamp, diff[:, vz_idx], color="green")
    plt.xlim(timestamp[0], timestamp[-1])
    plt.ylabel("vz")
    plt.grid("on")

    # Plot control inputs
    plt.subplots(figsize=(20, 5))
    label_names = ["T1 [N]", "T2 [N]", "T3 [N]", "T4 [N]", "S1 [rad]", "S2 [rad]", "S3 [rad]", "S4 [rad]"]
    for dim in range(control.shape[1]):
        plt.subplot(control.shape[1], 1, dim + 1)
        plt.plot(timestamp, control[:, dim])
        plt.xlim(timestamp[0], timestamp[-1])
        plt.ylabel(label_names[dim])
        plt.grid("on")
        if dim == 0:
            plt.title("Control")
        if dim != state_in.shape[1] - 1:
            ax = plt.gca()
            ax.axes.xaxis.set_ticklabels([])

    ###################### COMPARISON ######################
    # Parallel tracking simulation of state trajectory with real data
    if (has_ref or has_ref) and False:
        # --- Initialize controller ---
        print("Start control comparison based on recorded state_in with MPC...")
        model_options["only_use_nominal"] = True
        rtnmpc_nominal = NeuralNMPC(model_options, solver_options, sim_options, run_options)
        ocp_solver_nominal = rtnmpc_nominal.get_ocp_solver()

        model_options["only_use_nominal"] = False
        model_options["plus_neural"] = True
        model_options["minus_neural"] = False
        rtnmpc_neural = NeuralNMPC(model_options, solver_options, sim_options, run_options)
        ocp_solver_neural = rtnmpc_neural.get_ocp_solver()

        reference_generator = rtnmpc_nominal.get_reference_generator()

        nx = rtnmpc_nominal.get_acados_model().x.shape[0]
        nu = rtnmpc_nominal.get_acados_model().u.shape[0]
        rec_dict = {
            "timestamp": np.zeros((0, 1)),
            "comp_time_nominal": np.zeros((0, 1)),
            "comp_time_neural": np.zeros((0, 1)),
            "state_ref": np.zeros((0, nx)),
            "control_ref": np.zeros((0, nu)),
            "control_nominal": np.zeros((0, nu)),
            "control_neural": np.zeros((0, nu)),
        }

        # --- Simulate ---
        # Initialize state
        for t in range(state_in.shape[0]):
            rec_dict["timestamp"] = np.vstack((rec_dict["timestamp"], timestamp[t]))

            # Track current reference
            pos_ref_curr = pos_ref[t, :]
            quat_ref_curr = quat_ref[t, :]

            state_ref, control_ref = reference_generator.compute_trajectory(
                target_xyz=list(pos_ref_curr), target_qwxyz=list(quat_ref_curr)
            )

            rtnmpc_nominal.track(ocp_solver_nominal, state_ref, control_ref)
            rtnmpc_neural.track(ocp_solver_neural, state_ref, control_ref)

            # Set parameters
            for j in range(ocp_solver_nominal.N + 1):
                ocp_solver_nominal.set(j, "p", rtnmpc_nominal.acados_parameters[j, :])
                ocp_solver_neural.set(j, "p", rtnmpc_neural.acados_parameters[j, :])

            # Solve OCP with neural model
            comp_time_nominal_start = time.time()
            u_cmd_nominal = ocp_solver_nominal.solve_for_x0(state_in[t, :])
            rec_dict["comp_time_nominal"] = time.time() - comp_time_nominal_start

            comp_time_neural_start = time.time()
            u_cmd_neural = ocp_solver_neural.solve_for_x0(state_in[t, :])
            rec_dict["comp_time_neural"] = time.time() - comp_time_neural_start

            rec_dict["control_ref"] = np.append(rec_dict["control_ref"], control_ref[np.newaxis, 0], axis=0)
            rec_dict["control_nominal"] = np.append(rec_dict["control_nominal"], u_cmd_nominal[np.newaxis, :], axis=0)
            rec_dict["control_neural"] = np.append(rec_dict["control_neural"], u_cmd_neural[np.newaxis, :], axis=0)

        # Plot control features
        fig, _ = plt.subplots(figsize=(20, 5))
        for dim in range(rec_dict["control_nominal"].shape[1] - 1, -1, -1):
            plt.subplot(rec_dict["control_nominal"].shape[1], 1, dim + 1)
            plt.plot(timestamp, rec_dict["control_ref"][:, dim], "b--", label="Reference", alpha=0.5)
            plt.plot(timestamp, rec_dict["control_nominal"][:, dim], label="Nominal")
            plt.plot(timestamp, rec_dict["control_neural"][:, dim], label="Neural")
            plt.xlim(timestamp[0], timestamp[-1])
            plt.ylabel(f"Control {dim}")
            plt.grid("on")
            if dim == 0:
                plt.title("Control input (Reference vs. Nominal vs. Neural)")
                plt.legend()
            if dim != state_in.shape[1] - 1:
                ax = plt.gca()
                ax.axes.xaxis.set_ticklabels([])

    plt.show()
    halt = 1


if __name__ == "__main__":
    main()
