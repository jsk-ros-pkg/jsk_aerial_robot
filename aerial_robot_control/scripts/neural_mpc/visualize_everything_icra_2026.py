import sys, os
import torch
import numpy as np
import pandas as pd
import scienceplots  # DONT DELETE!
from config.configurations import EnvConfig, ModelFitConfig, DirectoryConfig
from utils.data_utils import undo_jsonify
import matplotlib.pyplot as plt
from utils.geometry_utils import v_dot_q, quaternion_inverse
from sim_environment.forward_prop import init_forward_prop
from utils.model_utils import load_model
from neural_controller import NeuralMPC
import time

sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from nmpc.nmpc_tilt_mt.tilt_qd import phys_param_beetle_omni as phys_omni


class struct(object):
    pass


def main():
    data_dir = DirectoryConfig.DATA_DIR

    # ds = "NMPCTiltQdServo_residual_dataset_04/dataset_001.csv"
    # ds = "NMPCTiltQdServo_real_machine_dataset_01/dataset_013.csv"
    # ds = "NMPCTiltQdServo_real_machine_dataset_VAL_FOR_PAPER/dataset_003.csv"
    # VAL: 1 (base), 3 (with ref)
    ds = "NMPCTiltQdServo_real_machine_dataset_GROUND_EFFECT_ONLY/dataset_001.csv"

    dataset_file = os.path.join(data_dir, ds)
    df = pd.read_csv(dataset_file)
    vz_idx = 5
    q_idx = 6
    state_feats = np.array([2, 3, 4, 5])  # , 6, 7, 8, 9])  # , 10, 11, 12])
    # state_feats = np.array([2, 3, 4, 5, 6, 7, 8, 9])  # , 10, 11, 12])
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

    # Format data
    timestamp = df["timestamp"].to_numpy() - df["timestamp"].to_numpy()[0]
    state_in = undo_jsonify(df["state_in"].to_numpy())
    state_out = undo_jsonify(df["state_out"].to_numpy())
    state_prop = undo_jsonify(df["state_prop"].to_numpy())
    control = undo_jsonify(df["control"].to_numpy())
    dt = df["dt"].to_numpy()[:, np.newaxis]
    if "position_ref" in df.columns and "quaternion_ref" in df.columns:
        has_ref = True
        pos_ref = undo_jsonify(df["position_ref"].to_numpy())
        quat_ref = undo_jsonify(df["quaternion_ref"].to_numpy())

    diff = (state_out - state_prop) / dt

    ##################################################################
    # PRUNE
    if ModelFitConfig.prune:
        x_vel_idx = np.array([vz_idx - 2, vz_idx - 1, vz_idx])
        y_vel_idx = np.array([vz_idx - 2, vz_idx - 1, vz_idx])

        if set(np.array(y_vel_idx)).issubset(set(y_reg_dims)):
            y_vel_idx_real = np.where(np.in1d(y_reg_dims, y_vel_idx))[0]
        elif set([vz_idx]).issubset(set(y_reg_dims)):
            y_vel_idx_real = np.where(np.in1d(y_reg_dims, vz_idx))[0]

        x = state_in[:, x_vel_idx]
        y = diff[:, y_vel_idx_real]

        histogram_bins = ModelFitConfig.histogram_n_bins  # Number of bins to use for histogram
        histogram_threshold = (
            ModelFitConfig.histogram_thresh
        )  # Remove bins where the total ratio of data is lower than this threshold
        velocity_cap = ModelFitConfig.vel_cap  # Also remove datasets point if abs(velocity) > x_cap

        prune_idx = prune_dataset(x, y, velocity_cap, histogram_bins, histogram_threshold, plot=False)
        timestamp = timestamp[prune_idx]
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
        p_traj = state_sequence[:, :3]
        v_w_traj = state_sequence[:, 3:6]
        q_traj = state_sequence[:, 6:10]
        other_traj = state_sequence[:, 10:]  # w, a_s, f_s, etc.

        v_b_traj = np.empty_like(v_w_traj)
        for t in range(len(v_w_traj)):
            v_b_traj[t, :] = v_dot_q(v_w_traj[t, :], quaternion_inverse(q_traj[t, :]))
        return np.concatenate((p_traj, v_b_traj, q_traj, other_traj), axis=1)

    if mlp_metadata["ModelFitConfig"]["input_transform"]:
        state_in_mlp_in = velocity_mapping(state_in)
    else:
        state_in_mlp_in = state_in.copy()

    if mlp_metadata["ModelFitConfig"]["label_transform"]:
        state_out_mlp_out = velocity_mapping(state_out)
        state_prop_mlp_out = velocity_mapping(state_prop)
    else:
        state_out_mlp_out = state_out.copy()
        state_prop_mlp_out = state_prop.copy()

    diff = (state_out - state_prop) / dt

    device = "cpu"  # "cuda"
    state_in_mlp_in_tensor = torch.from_numpy(state_in_mlp_in).type(torch.float32).to(torch.device(device))
    control_tensor = torch.from_numpy(control).type(torch.float32).to(torch.device(device))
    mlp_in = torch.cat((state_in_mlp_in_tensor[:, state_feats], control_tensor[:, u_feats]), axis=1)

    # Forward call
    mlp_out = neural_model(mlp_in)
    mlp_out = mlp_out.detach().cpu().numpy()

    # Unpack prediction outputs. Transform back to world reference frame
    if mlp_metadata["ModelFitConfig"]["label_transform"]:
        for t in range(state_in.shape[0]):
            mlp_out[t, :] = v_dot_q(mlp_out[t, :], state_in[t, q_idx : q_idx + 4])

    # ================ Model output vs. label plot ================
    plt.style.use(["science", "grid"])
    plt.rcParams.update({"font.size": 11})
    label_size = 14
    figsize = (7, 9)

    # Plot true labels vs. actual regression
    def plot_from_to(time_start, time_end):
        idx_start = np.argmin(np.abs(timestamp - time_start))
        idx_end = np.argmin(np.abs(timestamp - time_end))

        y = mlp_out
        y_true = diff

        plt.subplots(figsize=figsize)

        plt.subplot(3, 1, 1)
        # plt.title("Model output vs. label")
        # plt.plot(timestamp[idx_start:idx_end], y[idx_start:idx_end, 0] - y_true[idx_start:idx_end, 3], label="error", color="r", linestyle="--", alpha=0.5)
        plt.plot(timestamp[idx_start:idx_end], y_true[idx_start:idx_end, 3], label=r"$\boldsymbol{\tilde{y}}$")
        plt.plot(
            timestamp[idx_start:idx_end],
            y[idx_start:idx_end, 0],
            label=r"$\boldsymbol{f}_\mathrm{Neural}$",
            color="orange",
        )
        plt.xlim(timestamp[idx_start], timestamp[idx_end - 1])
        plt.ylabel("$v_x$ [m/s]", fontsize=label_size)
        plt.legend(fontsize=label_size)
        plt.grid("on")
        ax = plt.gca()
        ax.axes.xaxis.set_ticklabels([])

        plt.subplot(3, 1, 2)
        # plt.plot(timestamp[idx_start:idx_end], y[idx_start:idx_end, 1] - y_true[idx_start:idx_end, 4], color="r", linestyle="--", alpha=0.5)
        plt.plot(timestamp[idx_start:idx_end], y_true[idx_start:idx_end, 4])
        plt.plot(timestamp[idx_start:idx_end], y[idx_start:idx_end, 1], color="orange")
        plt.xlim(timestamp[idx_start], timestamp[idx_end - 1])
        plt.ylabel("$v_y$ [m/s]", fontsize=label_size)
        plt.grid("on")
        ax = plt.gca()
        ax.axes.xaxis.set_ticklabels([])

        plt.subplot(3, 1, 3)
        # plt.plot(timestamp[idx_start:idx_end], y[idx_start:idx_end, 2] - y_true[idx_start:idx_end, 5], color="r", linestyle="--", alpha=0.5)
        plt.plot(timestamp[idx_start:idx_end], y_true[idx_start:idx_end, 5])
        plt.plot(timestamp[idx_start:idx_end], y[idx_start:idx_end, 2], color="orange")
        plt.xlim(timestamp[idx_start], timestamp[idx_end - 1])
        plt.xlabel("$t$ [s]", fontsize=label_size)
        plt.ylabel("$v_z$ [m/s]", fontsize=label_size)
        plt.grid("on")

        # ================ Compensation ================
        mpc = struct()
        mpc.tilt = True
        mpc.include_servo_model = True
        mpc.include_thrust_model = False
        mpc.include_servo_derivative = False
        mpc.include_cog_dist_parameter = True
        mpc.phys = struct()

        mpc.phys = phys_omni

        # Define nominal model
        dynamics = init_forward_prop(mpc, return_continuous=True)

        # Compute linear acceleration with nominal model
        x_dot = np.empty(state_in.shape)
        for t in range(state_in.shape[0]):
            x_dot[t, :] = np.array(dynamics(x=state_in[t, :], u=control[t, :])["x_dot"]).squeeze()
        lin_acc = x_dot[:, 3:6]

        # Plot simulation results for acceleration and the effect of the neural model
        # idx_start, idx_end = 19000, 19501
        plt.subplots(figsize=figsize)
        plt.subplot(3, 1, 1)
        # plt.title("Neural Compensation on Linear Acceleration")
        plt.plot(timestamp[idx_start:idx_end], lin_acc[idx_start:idx_end, 0], label=r"Nominal model $\boldsymbol{a}$")
        plt.plot(
            timestamp[idx_start:idx_end],
            lin_acc[idx_start:idx_end, 0] + y[idx_start:idx_end, 0],
            label=r"Neural Compensation $\boldsymbol{a} + \boldsymbol{f}_{\mathrm{Neural}}$",
            color="orange",
        )
        plt.xlim(timestamp[idx_start], timestamp[idx_end - 1])
        plt.ylabel("$a_x$ [m/s$^2$]", fontsize=label_size)
        plt.legend(fontsize=label_size)
        plt.grid("on")
        ax = plt.gca()
        ax.axes.xaxis.set_ticklabels([])

        plt.subplot(3, 1, 2)
        plt.plot(timestamp[idx_start:idx_end], lin_acc[idx_start:idx_end, 1])
        plt.plot(timestamp[idx_start:idx_end], lin_acc[idx_start:idx_end, 1] + y[idx_start:idx_end, 1], color="orange")
        plt.xlim(timestamp[idx_start], timestamp[idx_end - 1])
        plt.ylabel("$a_y$ [m/s$^2$]", fontsize=label_size)
        plt.grid("on")
        ax = plt.gca()
        ax.axes.xaxis.set_ticklabels([])

        plt.subplot(3, 1, 3)
        plt.plot(timestamp[idx_start:idx_end], lin_acc[idx_start:idx_end, 2])
        plt.plot(timestamp[idx_start:idx_end], lin_acc[idx_start:idx_end, 2] + y[idx_start:idx_end, 2], color="orange")
        plt.xlim(timestamp[idx_start], timestamp[idx_end - 1])
        plt.xlabel("$t$ [s]", fontsize=label_size)
        plt.ylabel("$a_z$ [m/s$^2$]", fontsize=label_size)
        plt.grid("on")

    # Total
    time_start, time_end = 0, 400
    plot_from_to(time_start, time_end)
    # Hovering / Ground effect
    time_start, time_end = 30, 100
    plot_from_to(time_start, time_end)
    # Rotating
    time_start, time_end = 160, 270
    plot_from_to(time_start, time_end)

    plt.show()


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


if __name__ == "__main__":
    main()
