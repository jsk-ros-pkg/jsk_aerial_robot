import os, torch
import numpy as np
import pandas as pd
from config.configurations import EnvConfig
from utils.data_utils import undo_jsonify
import matplotlib.pyplot as plt
from utils.geometry_utils import v_dot_q, quaternion_inverse
from sim_environment.forward_prop import init_forward_prop
from utils.model_utils import load_model


class struct(object):
    pass


def main():
    RTNMPC = False
    OWN = True

    if RTNMPC and OWN:
        raise ValueError("Choose only one method to visualize.")
    if RTNMPC:
        df = pd.read_csv("/home/johannes/ros/neural-mpc/ros_dd_mpc/data/simplified_sim_dataset/train/dataset_001.csv")
        vz_idx = 9
        q_idx = 3
        state_feats = np.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12])
        u_feats = np.array([0, 1, 2, 3])
        y_reg_dims = np.array([7, 8, 9])
    elif OWN:
        df = pd.read_csv(
            "/home/johannes/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/aerial_robot_control/scripts/neural_nmpc/data/NMPCTiltQdServo_residual_dataset_03/dataset_002.csv"
        )
        vz_idx = 5
        q_idx = 6
        state_feats = np.array([2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12])
        u_feats = np.array([0, 1, 2, 3, 4, 5, 6, 7])
        y_reg_dims = np.array([3, 4, 5])

    # Call data
    state_in = undo_jsonify(df["state_in"].to_numpy())
    state_out = undo_jsonify(df["state_out"].to_numpy())
    if RTNMPC:
        state_prop = undo_jsonify(df["state_pred"].to_numpy())
        control = undo_jsonify(df["input_in"].to_numpy())
    elif OWN:
        state_prop = undo_jsonify(df["state_prop"].to_numpy())
        control = undo_jsonify(df["control"].to_numpy())
    dt = df["dt"].to_numpy()[:, np.newaxis]

    diff = (state_out - state_prop) / dt

    ##################################################################
    # PRUNE
    if RTNMPC:
        histogram_bins = 40  # Cluster data using histogram binning
        histogram_threshold = 0.001  # Remove bins where the total ratio of data is lower than this threshold
        velocity_cap = 16  # Also remove datasets point if abs(velocity) > x_cap

        prune_idx = prune_dataset(state_in, diff, velocity_cap, histogram_bins, histogram_threshold, plot=False)
        state_in = state_in[prune_idx, :]
        state_out = state_out[prune_idx, :]
        state_prop = state_prop[prune_idx, :]
        control = control[prune_idx, :]
        dt = dt[prune_idx, :]
        diff = diff[prune_idx, :]
    ##################################################################

    ##################################################################
    # MLP input
    def velocity_mapping(state_sequence):
        if RTNMPC:
            p_traj = state_sequence[:, :3]
            v_w_traj = state_sequence[:, 7:10]
            q_traj = state_sequence[:, 3:7]
            other_traj = state_sequence[:, 10:]  # w

            v_b_traj = np.empty_like(v_w_traj)
            for t in range(len(v_w_traj)):
                v_b_traj[t, :] = v_dot_q(v_w_traj[t, :], quaternion_inverse(q_traj[t, :]))
            return np.concatenate((p_traj, q_traj, v_b_traj, other_traj), axis=1)

        elif OWN:
            p_traj = state_sequence[:, :3]
            v_w_traj = state_sequence[:, 3:6]
            q_traj = state_sequence[:, 6:10]
            other_traj = state_sequence[:, 10:]  # w, a_s, f_s, etc.

            v_b_traj = np.empty_like(v_w_traj)
            for t in range(len(v_w_traj)):
                v_b_traj[t, :] = v_dot_q(v_w_traj[t, :], quaternion_inverse(q_traj[t, :]))
            return np.concatenate((p_traj, v_b_traj, q_traj, other_traj), axis=1)

    state_in_mlp_in = velocity_mapping(state_in)
    state_out_mlp_in = velocity_mapping(state_out)
    state_prop_mlp_in = velocity_mapping(state_prop)
    diff_mlp_in = (state_out_mlp_in - state_prop_mlp_in) / dt

    if RTNMPC:
        device = "cpu"
    elif OWN:
        device = "cuda"
    state_in_mlp_in_tensor = torch.from_numpy(state_in_mlp_in).type(torch.float32).to(torch.device(device))
    control_tensor = torch.from_numpy(control).type(torch.float32).to(torch.device(device))
    mlp_in = torch.cat((state_in_mlp_in_tensor[:, state_feats], control_tensor[:, u_feats]), axis=1)

    plt.figure(figsize=(20, 5))
    plt.title("State Out MLP In - State Pred MLP In")
    plt.subplot(2, 1, 1)
    plt.plot(state_out_mlp_in[:, vz_idx], label="state_out_mlp_in")
    plt.plot(state_prop_mlp_in[:, vz_idx], label="state_prop_mlp_in")
    plt.ylabel("vz")
    plt.legend()
    plt.grid("on")
    plt.subplot(2, 1, 2)
    plt.plot(diff_mlp_in[:, vz_idx], label="(state_out_mlp_in - state_pred_mlp_in) / dt", color="green")
    plt.ylabel("vz")
    plt.legend()
    plt.grid("on")

    plt.figure(figsize=(20, 5))
    plt.title("State Out MLP In - State Pred MLP In")
    plt.subplot(2, 1, 1)
    plt.plot(state_in_mlp_in[:, 2], label="state_in_mlp_in")
    plt.ylabel("z")
    plt.legend()
    plt.grid("on")
    plt.subplot(2, 1, 2)
    plt.plot(diff_mlp_in[:, vz_idx], label="(state_out_mlp_in - state_pred_mlp_in) / dt", color="green")
    plt.ylabel("vz")
    plt.legend()
    plt.grid("on")
    ##################################################################

    ##################################################################
    # MLP output
    if RTNMPC:
        # Define model
        from ml_casadi.torch.modules.nn import MultiLayerPerceptron
        from ml_casadi.torch.modules.module import TorchMLCasadiModule

        class NormalizedMLP(TorchMLCasadiModule):
            def __init__(self, model, x_mean, x_std, y_mean, y_std):
                super().__init__()
                self.model = model
                self.input_size = self.model.input_size
                self.output_size = self.model.output_size
                self.register_buffer("x_mean", x_mean)
                self.register_buffer("x_std", x_std)
                self.register_buffer("y_mean", y_mean)
                self.register_buffer("y_std", y_std)

            def forward(self, x):
                return (self.model((x - self.x_mean) / self.x_std) * self.y_std) + self.y_mean

        directory, file_name = (
            "/home/johannes/ros/neural-mpc/ros_dd_mpc/results/model_fitting/5f15661/simple_sim_mlp",
            "drag__motor_noise__noisy__no_payload.pt",
        )
        saved_dict = torch.load(os.path.join(directory, f"{file_name}"))
        mlp_model = MultiLayerPerceptron(
            saved_dict["input_size"],
            saved_dict["hidden_size"],
            saved_dict["output_size"],
            saved_dict["hidden_layers"],
            "Tanh",
        )
        neural_model = NormalizedMLP(
            mlp_model,
            torch.tensor(np.zeros((saved_dict["input_size"],))).float(),
            torch.tensor(np.zeros((saved_dict["input_size"],))).float(),
            torch.tensor(np.zeros((saved_dict["output_size"],))).float(),
            torch.tensor(np.zeros((saved_dict["output_size"],))).float(),
        )

        # Load model
        neural_model.load_state_dict(saved_dict["state_dict"])
        neural_model.eval()

    elif OWN:
        # Define and load model
        model_options = EnvConfig.model_options
        sim_options = EnvConfig.sim_options
        run_options = EnvConfig.run_options
        neural_model, mlp_metadata = load_model(model_options, sim_options, run_options)
        neural_model.eval()

        # ==============>
        # # Transform velocity of state to Body frame
        # state_b = np.zeros(state_in.shape)
        # for t in range(state_in.shape[0]):
        #     v_b = v_dot_q(state_in[t, 3:6], quaternion_inverse(state_in[t, 6:10]))
        #     state_b[t] = np.concatenate((state_in[t, :3], v_b, state_in[t, 6:]), axis=0)
        # # Compute forward pass
        # y = np.zeros((state_in.shape[0], 3)).astype(np.float32)
        # for t in range(state_in.shape[0]):
        #     # Assemble input
        #     s_b = torch.from_numpy(state_b[t, state_feats]).type(torch.float32).to(torch.device("cuda"))
        #     u = torch.from_numpy(control[t, u_feats]).type(torch.float32).to(torch.device("cuda"))
        #     mlp_in = torch.cat((s_b, u)).unsqueeze(0)  # Add batch dimension
        #     # Forward call MLP
        #     mlp_out = neural_model(mlp_in).cpu().detach().numpy() / dt[t]
        #     # Transform velocity back to world frame
        #     if set([3, 4, 5]).issubset(set(y_reg_dims)):
        #         v_idx = np.where(y_reg_dims == 3)[0][0]  # Assumed that v_x, v_y, v_z are consecutively in output
        #         v_b = mlp_out[v_idx : v_idx + 3]
        #         v_w = v_dot_q(v_b.T, state_in[t, 6:10]).T
        #         mlp_out = np.concatenate((mlp_out[:, :v_idx], v_w, mlp_out[:, v_idx + 3 :]), axis=1)
        #     elif set([4, 5]).issubset(set(y_reg_dims)):
        #         v_idx = np.where(y_reg_dims == 4)[0][0]  # Assumed that v_y, v_z are consecutively in output
        #         v_b = np.append(0, mlp_out[v_idx : v_idx + 2])
        #         v_w = v_dot_q(v_b.T, state_in[t, 6:10]).T
        #         mlp_out = np.concatenate((mlp_out[:, :v_idx], v_w, mlp_out[:, v_idx + 2 :]), axis=1)
        #     elif set([5]).issubset(set(y_reg_dims)):
        #         # Predict only v_z so set v_x and v_y to 0 in Body frame and then transform to World frame
        #         # The predicted v_z therefore also has influence on the x and y velocities in World frame
        #         # Adjust mapping later on
        #         v_idx = np.where(y_reg_dims == 5)[0][0]
        #         v_b = np.append(np.array([0, 0]), mlp_out[v_idx])
        #         v_w = v_dot_q(v_b.T, state_in[t, 6:10])[:, np.newaxis]
        #         mlp_out = np.concatenate((mlp_out[:v_idx], v_w, mlp_out[v_idx + 1 :]))
        #     y[t, :] = np.squeeze(mlp_out)
    # ==============<

    # Forward call
    mlp_out = neural_model(mlp_in)
    mlp_out = mlp_out.detach().cpu().numpy()
    if RTNMPC:
        mlp_out = mlp_out[:, y_reg_dims]
    elif OWN:
        mlp_out = mlp_out  # / dt

    # Unpack prediction outputs. Transform back to world reference frame
    if RTNMPC and (y_reg_dims != np.array([7, 8, 9])).all() or OWN and (y_reg_dims != np.array([3, 4, 5])).all():
        raise NotImplementedError("Only implemented for vx, vy, vz output.")

    if RTNMPC or (OWN and mlp_metadata["ModelFitConfig"]["label_transform"]):
        for t in range(state_in_mlp_in.shape[0]):
            mlp_out[t, :] = v_dot_q(mlp_out[t, :], state_in_mlp_in[t, q_idx : q_idx + 4])

    # Plot true labels vs. actual regression
    y = mlp_out
    y_true = diff
    plt.subplots(figsize=(10, 5))
    plt.title("Model output vs. label")
    for i, dim in enumerate(y_reg_dims):
        plt.subplot(y.shape[1], 1, i + 1)
        plt.plot(y[:, i], label="y_regressed")
        plt.plot(y[:, i] - y_true[:, dim], label="error", color="r", linestyle="--", alpha=0.5)
        plt.plot(y_true[:, dim], label="y_true", color="orange")
        plt.ylabel(f"D{dim}")
        plt.legend()
        plt.grid("on")
        if i != y.shape[1] - 1:
            ax = plt.gca()
            ax.axes.xaxis.set_ticklabels([])

    # Plot loss per dimension
    loss = np.square(y_true[:, y_reg_dims] - y)
    plt.subplots(figsize=(10, 5))
    plt.title("Neural Model Loss per Dimension")
    for i, dim in enumerate(y_reg_dims):
        plt.subplot(y.shape[1], 1, i + 1)
        plt.plot(loss[:, i], color="red")
        plt.plot(
            [0, loss.shape[0]],
            [np.mean(loss[:, i]), np.mean(loss[:, i])],
            color="blue",
            linestyle="--",
            label=f"Mean = {np.mean(loss[:, i]):.6f}",
        )
        plt.legend()
        plt.ylabel(f"Loss D{dim}")
        plt.grid("on")
        if i != y.shape[1] - 1:
            ax = plt.gca()
            ax.axes.xaxis.set_ticklabels([])

    # Plot total loss and RMSE
    plt.figure(figsize=(10, 5))
    total_loss = np.sum(loss, axis=1)
    rmse = np.sqrt(total_loss / y.shape[1])
    plt.plot(total_loss, label="Total Loss", color="red")
    plt.plot(rmse, label="RMSE", color="green")
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
    plt.xlabel("Time [s]")
    plt.ylabel("Loss / RMSE")
    plt.title("Neural Model Total Loss and RMSE")
    plt.legend()
    plt.grid("on")

    ##################################################################
    # Simulate intermediate acceleration before integration
    if RTNMPC:
        motor_noise = True
        noisy = True
        drag = True
        max_input_value = 1
        min_input_value = 0
        max_thrust = 20
        mass = 1.0
        rotor_drag_xy = 0.3
        rotor_drag_z = 0.0  # No rotor drag in the z dimension
        rotor_drag = np.array([rotor_drag_xy, rotor_drag_xy, rotor_drag_z])
        aero_drag = 0.08
        g = np.array([0.0, 0.0, 9.81])

        def v_dynamics(u, q, f_d):
            f_thrust = u * max_thrust
            a_thrust = np.array([0.0, 0.0, f_thrust[0] + f_thrust[1] + f_thrust[2] + f_thrust[3]]) / mass

            v_dynamics = v_dot_q(a_thrust + f_d / mass, q) - g
            return v_dynamics

        lin_acc = np.zeros((state_in.shape[0], 3))
        lin_acc_dist = np.zeros((state_in.shape[0], 3))
        for t in range(state_in.shape[0]):
            q = state_in[t, 3:7]
            v_w = state_in[t, 7:10]
            u_raw = control[t, :]

            # Motor noise disturbance (resimulate)
            if motor_noise:
                u_dist = np.zeros((control.shape[1],))
                for i, u_i in enumerate(u_raw):
                    std = 0.02 * np.sqrt(u_i)
                    noise_u = np.random.normal(loc=0.1 * (u_i / 1.3) ** 2, scale=std)
                    u_dist[i] = max(min(u_i - noise_u, max_input_value), min_input_value)
            else:
                u_dist = u_raw

            # CoG disturbance (resimulate)
            if noisy:
                f_d = np.random.normal(size=(3,), scale=10 * np.mean(dt))
            else:
                f_d = np.zeros((3,))

            # Drag disturbance (resimulate)
            if drag:
                # Compute aerodynamic drag in body frame
                v_b = v_dot_q(v_w, quaternion_inverse(q))
                a_drag = -aero_drag * v_b**2 * np.sign(v_b) / mass
                a_drag -= rotor_drag * v_b / mass
                # Transform drag acceleration to world frame
                a_drag = v_dot_q(a_drag, q)
            else:
                a_drag = np.zeros((3,))

            # Compute linear acceleration with nominal model
            lin_acc[t, :] = v_dynamics(u_raw, q, np.zeros((3,)))
            lin_acc_dist[t, :] = v_dynamics(u_dist, q, f_d) + a_drag

    if OWN:
        # Replicate important properties for nominal model
        nmpc = struct()
        nmpc.tilt = True
        nmpc.include_servo_model = True
        nmpc.include_thrust_model = False
        nmpc.include_servo_derivative = False
        nmpc.include_cog_dist_parameter = True
        nmpc.phys = struct()

        import sys, os

        sys.path.append(os.path.dirname(os.path.dirname(__file__)))
        from nmpc.nmpc_tilt_mt.archive import phys_param_beetle_art as phys_art

        nmpc.phys = phys_art

        # Define nominal model
        dynamics, _, _ = init_forward_prop(nmpc)

        # Compute linear acceleration with nominal model
        x_dot = np.empty(state_in.shape)
        for t in range(state_in.shape[0]):
            x_dot[t, :] = np.array(dynamics(x=state_in[t, :], u=control[t, :])["x_dot"]).squeeze()
        lin_acc = x_dot[:, 3:6]

        # CoG disturbance (resimulate)
        cog_dist = np.zeros((control.shape[0], 6))
        for t in range(control.shape[0]):
            u_cmd = control[t, :]
            max_thrust = np.average(u_cmd[:4])
            # Ground effect increases lift the closer drone is to the ground
            # Force values behave in [-thrust_max, thrust_max]
            z = state_in[t, 2]
            cog_dist_factor = 0.4
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
    plt.subplots(figsize=(10, 5))
    plt.title("Model Output")
    for i, dim in enumerate(y_reg_dims):
        plt.subplot(len(y_reg_dims), 1, i + 1)
        plt.plot(lin_acc[:, i], label="Acceleration by undisturbed model", color="tab:blue")
        plt.plot(lin_acc_dist[:, i], label="Acceleration by disturbed model", color="tab:olive")
        plt.plot(lin_acc_dist[:, i] + y[:, i], label="Neural Compensation (+)", color="tab:orange")
        plt.plot(lin_acc_dist[:, i] - y[:, i], label="Neural Compensation (-)", color="tab:brown")
        plt.ylabel(f"D{dim}")
        if i == 0:
            plt.legend()
        plt.grid("on")
        if i != y.shape[1] - 1:
            ax = plt.gca()
            ax.axes.xaxis.set_ticklabels([])

    ##################################################################

    # Plot input state features
    plt.subplots(figsize=(20, 5))
    plt.title("State In & State Out")
    for dim in range(state_in.shape[1]):
        plt.subplot(state_in.shape[1], 2, dim * 2 + 1)
        plt.plot(state_in[:, dim], label="state_in")
        plt.plot(state_out[:, dim], label="state_out")
        plt.plot(state_prop[:, dim], label="state_prop")
        plt.ylabel(f"Feature {dim}")
        if dim == 0:
            plt.legend()
        plt.grid("on")

    # Plot residual dynamics
    for dim in range(state_in.shape[1]):
        plt.subplot(state_in.shape[1], 2, dim * 2 + 2)
        plt.plot(diff[:, dim], color="green")
        if dim == 0:
            plt.title("State Out - State Pred")
        plt.grid("on")

    # Plot absolute velocity and its residual dynamics in z
    plt.figure(figsize=(20, 5))
    plt.title("State Out & State Pred")
    plt.subplot(2, 1, 1)
    plt.plot(state_out[:, vz_idx], label="state_out")
    plt.plot(state_prop[:, vz_idx], label="state_prop")
    plt.ylabel("vz")
    plt.legend()
    plt.grid("on")
    plt.subplot(2, 1, 2)
    plt.plot(diff[:, vz_idx], label="(state_out - state_pred) / dt", color="green")
    plt.ylabel("vz")
    plt.legend()
    plt.grid("on")

    plt.figure(figsize=(20, 5))
    plt.title("State In")
    plt.subplot(2, 1, 1)
    plt.plot(state_in[:, 2], label="state_in")
    plt.ylabel("z")
    plt.legend()
    plt.grid("on")
    plt.subplot(2, 1, 2)
    plt.plot(diff[:, vz_idx], label="(state_out - state_pred) / dt", color="green")
    plt.ylabel("vz")
    plt.legend()
    plt.grid("on")

    # Plot control inputs
    plt.subplots(figsize=(20, 5))
    plt.title("Control")
    for dim in range(control.shape[1]):
        plt.subplot(control.shape[1], 1, dim + 1)
        plt.plot(control[:, dim], label="control")
        if dim == 0:
            plt.legend()
        plt.grid("on")

    plt.tight_layout()
    plt.show()
    halt = 1


if __name__ == "__main__":
    main()


def prune_dataset(x, y, x_cap, bins, thresh, plot, labels=None):
    """
    Prunes the collected model error dataset with two filters. First, remove values where the input values (velocities)
    exceed 10. Second, create an histogram for each of the three axial velocity errors (y) with the specified number of
    bins and remove any data where the total amount of samples in that bin is less than the specified threshold ratio.
    :param x: dataset of input GP features (velocities). Dimensions N x n (N entries and n dimensions)
    :param y: dataset of errors. Dimensions N x m (N entries and m dimensions)
    :param x_cap: remove values from dataset if x > x_cap or x < -x_cap
    :param bins: number of bins used for histogram
    :param thresh: threshold ratio below which data from that bin will be removed
    :param plot: make a plot of the pruning
    :param labels: Labels to use for the plot
    :return: The indices kept after the pruning
    """

    n_bins = bins
    original_length = x.shape[0]

    plot_bins = []
    if plot:
        plt.figure()
        for i in range(y.shape[1]):
            plt.subplot(y.shape[1] + 1, 1, i + 1)
            h, bins = np.histogram(y[:, i], bins=n_bins)
            plot_bins.append(bins)
            plt.bar(bins[:-1], h, np.ones_like(h) * (bins[1] - bins[0]), align="edge", label="discarded")
            if labels is not None:
                plt.ylabel(labels[i])

    pruned_idx_unique = np.zeros(0, dtype=int)

    # Prune velocities (max axial velocity = x_cap m/s).
    if x_cap is not None:
        for i in range(x.shape[1]):
            pruned_idx = np.where(np.abs(x[:, i]) > x_cap)[0]
            pruned_idx_unique = np.unique(np.append(pruned_idx, pruned_idx_unique))

    # Prune by error histogram dimension wise (discard bins with less than 1% of the data)
    for i in range(y.shape[1]):
        h, bins = np.histogram(y[:, i], bins=n_bins)
        for j in range(len(h)):
            if h[j] / np.sum(h) < thresh:
                pruned_idx = np.where(np.logical_and(bins[j + 1] >= y[:, i], y[:, i] >= bins[j]))
                pruned_idx_unique = np.unique(np.append(pruned_idx, pruned_idx_unique))

    y_norm = np.sqrt(np.sum(y**2, 1))

    # Prune by error histogram norm
    h, error_bins = np.histogram(y_norm, bins=n_bins)
    h = h / np.sum(h)
    if plot:
        plt.subplot(y.shape[1] + 1, 1, y.shape[1] + 1)
        plt.bar(error_bins[:-1], h, np.ones_like(h) * (error_bins[1] - error_bins[0]), align="edge", label="discarded")

    for j in range(len(h)):
        if h[j] / np.sum(h) < thresh:
            pruned_idx = np.where(np.logical_and(error_bins[j + 1] >= y_norm, y_norm >= error_bins[j]))
            pruned_idx_unique = np.unique(np.append(pruned_idx, pruned_idx_unique))

    y = np.delete(y, pruned_idx_unique, axis=0)

    if plot:
        for i in range(y.shape[1]):
            plt.subplot(y.shape[1] + 1, 1, i + 1)
            h, bins = np.histogram(y[:, i], bins=plot_bins[i])
            plt.bar(bins[:-1], h, np.ones_like(h) * (bins[1] - bins[0]), align="edge", label="kept")
            plt.legend()

        plt.subplot(y.shape[1] + 1, 1, y.shape[1] + 1)
        h, bins = np.histogram(np.sqrt(np.sum(y**2, 1)), bins=error_bins)
        h = h / sum(h)
        plt.bar(bins[:-1], h, np.ones_like(h) * (bins[1] - bins[0]), align="edge", label="kept")
        plt.ylabel("Error norm")
        plt.show()

    kept_idx = np.delete(np.arange(0, original_length), pruned_idx_unique)
    return kept_idx
