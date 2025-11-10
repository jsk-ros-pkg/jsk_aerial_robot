import numpy as np
import torch
import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import

from utils.geometry_utils import v_dot_q, quaternion_inverse
from config.configurations import DirectoryConfig
from utils.data_utils import safe_mkdir_recursive
from sim_environment.forward_prop import init_forward_prop


def plot_trajectory_parallel(
    model_options, rec_dict, rtnmpc_nominal, rtnmpc_minus_neural, rtnmpc_plus_neural, dist_dict=None, save=False
):
    figures = []
    state_in_nominal = rec_dict["state_in_nominal"]
    state_in_minus_neural = rec_dict["state_in_minus_neural"]
    state_in_plus_neural = rec_dict["state_in_plus_neural"]
    state_out_nominal = rec_dict["state_out_nominal"]
    state_out_minus_neural = rec_dict["state_out_minus_neural"]
    state_out_plus_neural = rec_dict["state_out_plus_neural"]
    state_prop_nominal = rec_dict["state_prop_nominal"]
    state_prop_minus_neural = rec_dict["state_prop_minus_neural"]
    state_prop_plus_neural = rec_dict["state_prop_plus_neural"]
    control_nominal = rec_dict["control_nominal"]
    control_minus_neural = rec_dict["control_minus_neural"]
    control_plus_neural = rec_dict["control_plus_neural"]
    timestamp = rec_dict["timestamp"]

    # Plot state features
    fig, _ = plt.subplots(figsize=(20, 5))
    for dim in range(state_in_nominal.shape[1] - 1, -1, -1):
        plt.subplot(state_in_nominal.shape[1], 1, dim + 1)
        plt.plot(timestamp, state_in_nominal[:, dim], label="state_in_nominal", alpha=0.5)
        plt.plot(timestamp, state_out_nominal[:, dim], label="state_out_nominal")
        plt.plot(timestamp, state_prop_nominal[:, dim], label="state_prop_nominal", alpha=0.5)
        # plt.plot(timestamp, state_in_minus_neural[:, dim], label="state_in_minus_neural", alpha=0.5)
        # plt.plot(timestamp, state_out_minus_neural[:, dim], label="state_out_minus_neural")
        # plt.plot(timestamp, state_prop_minus_neural[:, dim], label="state_prop_minus_neural", alpha=0.5)
        plt.plot(timestamp, state_in_plus_neural[:, dim], label="state_in_plus_neural", alpha=0.5)
        plt.plot(timestamp, state_out_plus_neural[:, dim], label="state_out_plus_neural")
        plt.plot(timestamp, state_prop_plus_neural[:, dim], label="state_prop_plus_neural", alpha=0.5)
        plt.ylabel(f"D{dim}")
        if dim == 0:
            plt.title("State In & State Out")
            plt.legend()
        plt.grid("on")
        plt.xlim(timestamp[0], timestamp[-1])
        if dim != state_in_nominal.shape[1] - 1:
            ax = plt.gca()
            ax.axes.xaxis.set_ticklabels([])
    mng = plt.get_current_fig_manager()
    mng.resize(*mng.window.maxsize())
    figures.append(fig)

    # Plot control features
    fig, _ = plt.subplots(figsize=(20, 5))
    plt.title("Control input")
    for dim in range(control_nominal.shape[1] - 1, -1, -1):
        plt.subplot(control_nominal.shape[1], 1, dim + 1)
        plt.plot(timestamp, control_nominal[:, dim], label="control_nominal")
        # plt.plot(timestamp, control_minus_neural[:, dim], label="control_minus_neural")
        plt.plot(timestamp, control_plus_neural[:, dim], label="control_plus_neural")
        if dim == 0:
            plt.legend()
        plt.grid("on")
        plt.xlim(timestamp[0], timestamp[-1])
        if dim != state_in_nominal.shape[1] - 1:
            ax = plt.gca()
            ax.axes.xaxis.set_ticklabels([])
    mng = plt.get_current_fig_manager()
    mng.resize(*mng.window.maxsize())
    figures.append(fig)

    # Plot in vz feature
    fig = plt.figure(figsize=(20, 5))
    plt.plot(timestamp, state_in_nominal[:, 5], label="state_in_nominal")
    plt.plot(timestamp, state_out_nominal[:, 5], label="state_out_nominal")
    plt.plot(timestamp, state_prop_nominal[:, 5], label="state_prop_nominal")
    # plt.plot(timestamp, state_in_minus_neural[:, 5], label="state_in_minus_neural")
    # plt.plot(timestamp, state_out_minus_neural[:, 5], label="state_out_minus_neural")
    # plt.plot(timestamp, state_prop_minus_neural[:, 5], label="state_prop_minus_neural")
    plt.plot(timestamp, state_in_plus_neural[:, 5], label="state_in_plus_neural")
    plt.plot(timestamp, state_out_plus_neural[:, 5], label="state_out_plus_neural")
    plt.plot(timestamp, state_prop_plus_neural[:, 5], label="state_prop_plus_neural")
    plt.grid("on")
    plt.title("Dim 5 zoom in")
    plt.legend()
    plt.xlim(timestamp[0], timestamp[-1])
    mng = plt.get_current_fig_manager()
    mng.resize(*mng.window.maxsize())
    figures.append(fig)

    # Plot labels for neural network regression
    dt = np.expand_dims(rec_dict["dt"], 1)
    y_true_nominal = (state_out_nominal - state_prop_nominal) / dt
    # y_true_minus_neural = (state_out_minus_neural - state_prop_minus_neural) / dt
    y_true_plus_neural = (state_out_plus_neural - state_prop_plus_neural) / dt

    fig, _ = plt.subplots(figsize=(20, 5))
    plt.title("(State Out - State Pred) / dt")
    for dim in range(state_in_nominal.shape[1] - 1, -1, -1):
        plt.subplot(state_in_nominal.shape[1], 1, dim + 1)
        plt.plot(timestamp, y_true_nominal[:, dim], color="tab:red", label="nominal")
        # plt.plot(timestamp, y_true_minus_neural[:, dim], color="tab:blue", label="minus")
        plt.plot(timestamp, y_true_plus_neural[:, dim], color="tab:green", label="plus")
        if dim == 0:
            plt.legend()
        plt.grid("on")
        plt.xlim(timestamp[0], timestamp[-1])
        if dim != state_in_nominal.shape[1] - 1:
            ax = plt.gca()
            ax.axes.xaxis.set_ticklabels([])
    mng = plt.get_current_fig_manager()
    mng.resize(*mng.window.maxsize())
    figures.append(fig)

    #######################################################################
    # Simulate intermediate acceleration vector before integration
    dynamics, _, _ = init_forward_prop(rtnmpc_nominal)
    x_dot = np.empty(state_in_nominal.shape)
    for t in range(state_in_nominal.shape[0]):
        x_dot[t, :] = np.array(dynamics(x=state_in_nominal[t, :], u=control_nominal[t, :])["x_dot"]).squeeze()
    lin_acc_nominal = x_dot[:, 3:6]
    if rtnmpc_nominal.include_cog_dist_parameter:
        lin_acc_dist_nominal = lin_acc_nominal + dist_dict["cog_dist_nominal"][1::2, :3] / rtnmpc_nominal.phys.mass

    #######################################################################
    # Plot regression of neural network
    if rtnmpc_minus_neural.use_mlp:
        state_in_minus_neural = state_in_nominal.copy()
        # Transform velocity of state to Body frame
        state_b_nominal = np.zeros(state_in_nominal.shape)

        if rtnmpc_minus_neural.mlp_metadata["ModelFitConfig"]["input_transform"]:
            for t in range(state_in_nominal.shape[0]):
                v_b = v_dot_q(state_in_nominal[t, 3:6], quaternion_inverse(state_in_nominal[t, 6:10]))
                state_b_nominal[t, :] = np.concatenate((state_in_nominal[t, :3], v_b, state_in_nominal[t, 6:]), axis=0)

        state_b_torch_nominal = (
            torch.from_numpy(state_b_nominal[:, rtnmpc_minus_neural.state_feats])
            .type(torch.float32)
            .to(torch.device("cuda"))
        )
        control_torch_nominal = (
            torch.from_numpy(control_nominal[:, rtnmpc_minus_neural.u_feats])
            .type(torch.float32)
            .to(torch.device("cuda"))
        )
        mlp_in = torch.cat((state_b_torch_nominal, control_torch_nominal), dim=1)
        # Forward call MLP
        # rtnmpc_plus_neural = rtnmpc_minus_neural
        rtnmpc_minus_neural.neural_model.eval()
        mlp_out = rtnmpc_minus_neural.neural_model(mlp_in).cpu().detach().numpy()

        # Transform velocity back to world frame
        if rtnmpc_minus_neural.mlp_metadata["ModelFitConfig"]["label_transform"]:
            for t in range(state_in_minus_neural.shape[0]):
                if set([3, 4, 5]).issubset(set(rtnmpc_minus_neural.y_reg_dims)):
                    v_idx = np.where(rtnmpc_minus_neural.y_reg_dims == 3)[0][
                        0
                    ]  # Assumed that v_x, v_y, v_z are consecutive
                    v_b = mlp_out[t, v_idx : v_idx + 3]
                    v_w = v_dot_q(v_b.T, state_in_nominal[t, 6:10]).T
                    mlp_out[t, :] = np.concatenate((mlp_out[t, :v_idx], v_w, mlp_out[t, v_idx + 3 :]), axis=1)

        # Scale by weight
        if model_options["scale_by_weight"]:
            mlp_out /= rtnmpc_minus_neural.phys.mass

        # Plot true labels vs. actual regression
        y = mlp_out
        fig, _ = plt.subplots(figsize=(10, 5))
        plt.title("Model Output vs. Labels")
        for i, dim in enumerate(rtnmpc_minus_neural.y_reg_dims):
            plt.subplot(y.shape[1], 1, i + 1)
            plt.plot(timestamp, y[:, i], label="y_regressed", color="tab:blue")
            plt.plot(timestamp, y[:, i] - y_true_nominal[:, dim], label="error", color="r", linestyle="--", alpha=0.5)
            plt.plot(timestamp, y_true_nominal[:, dim], label="y_true", color="orange")
            plt.ylabel(f"D{dim}")
            if i == 0:
                plt.legend()
            plt.grid("on")
            plt.xlim(timestamp[0], timestamp[-1])
            if i != y.shape[1] - 1:
                ax = plt.gca()
                ax.axes.xaxis.set_ticklabels([])
        mng = plt.get_current_fig_manager()
        mng.resize(*mng.window.maxsize())
        figures.append(fig)

        # Plot loss per dimension
        loss = np.square(y_true_nominal[:, rtnmpc_minus_neural.y_reg_dims] - y)
        fig, _ = plt.subplots(figsize=(10, 5))
        plt.title("Neural Model Loss per Dimension")
        for i, dim in enumerate(rtnmpc_minus_neural.y_reg_dims):
            plt.subplot(y.shape[1], 1, i + 1)
            plt.plot(timestamp, loss[:, i], color="tab:red")
            plt.plot(
                [timestamp[0], timestamp[-1]],
                [np.mean(loss[:, i]), np.mean(loss[:, i])],
                color="tab:blue",
                linestyle="--",
                label=f"Mean = {np.mean(loss[:, i]):.6f}",
            )
            plt.legend()
            plt.ylabel(f"Loss D{dim}")
            plt.grid("on")
            plt.xlim(timestamp[0], timestamp[-1])
            if i != y.shape[1] - 1:
                ax = plt.gca()
                ax.axes.xaxis.set_ticklabels([])
        mng = plt.get_current_fig_manager()
        mng.resize(*mng.window.maxsize())
        figures.append(fig)

        # # Plot total loss and RMSE
        # fig = plt.figure(figsize=(10, 5))
        # plt.title("Neural Model Total Loss and RMSE")
        # total_loss = np.sum(loss, axis=1)
        # rmse = np.sqrt(total_loss / y_minus_neural.shape[1])
        # plt.plot(timestamp, total_loss, label="Total Loss", color="red")
        # plt.plot(timestamp, rmse, label="RMSE", color="green")
        # plt.plot(
        #     [timestamp[0], timestamp[-1]],
        #     [np.mean(total_loss), np.mean(total_loss)],
        #     color="tab:red",
        #     linestyle="--",
        #     alpha=0.7,
        #     label=f"Mean Total Loss = {np.mean(total_loss):.6f}",
        # )
        # plt.plot(
        #     [timestamp[0], timestamp[-1]],
        #     [np.mean(rmse), np.mean(rmse)],
        #     color="tab:green",
        #     linestyle="--",
        #     alpha=0.7,
        #     label=f"Mean RMSE = {np.mean(rmse):.6f}",
        # )
        # plt.xlabel("Time [s]")
        # plt.ylabel("Loss / RMSE")
        # plt.legend()
        # plt.grid("on")
        # plt.xlim(timestamp[0], timestamp[-1])
        # mng = plt.get_current_fig_manager()
        # mng.resize(*mng.window.maxsize())
        # figures.append(fig)

        # Simulate intermediate acceleration vector before integration
        # dynamics, _, _ = init_forward_prop(rtnmpc_minus_neural)
        # x_dot = np.empty(state_in_minus_neural.shape)
        # for t in range(state_in_minus_neural.shape[0]):
        #     x_dot[t, :] = np.array(
        #         dynamics(x=state_in_minus_neural[t, :], u=control_minus_neural[t, :])["x_dot"]
        #     ).squeeze()
        # lin_acc_neural = x_dot[:, 3:6]
        # lin_acc_dist_neural = (
        #     lin_acc_neural + dist_dict["cog_dist_neural"][1::2, :3] / rtnmpc_minus_neural.phys.mass
        # )

        # Simulate intermediate acceleration vector before integration
        dynamics, _, _ = init_forward_prop(rtnmpc_plus_neural)
        x_dot = np.empty(state_in_plus_neural.shape)
        for t in range(state_in_plus_neural.shape[0]):
            x_dot[t, :] = np.array(
                dynamics(x=state_in_plus_neural[t, :], u=control_plus_neural[t, :])["x_dot"]
            ).squeeze()
        lin_acc_plus_neural = x_dot[:, 3:6]
        if rtnmpc_nominal.include_cog_dist_parameter:
            lin_acc_dist_plus_neural = (
                lin_acc_plus_neural + dist_dict["cog_dist_plus_neural"][1::2, :3] / rtnmpc_plus_neural.phys.mass
            )

        # fig, _ = plt.subplots(figsize=(10, 5))
        # plt.title("Model Output (minus)")
        # for i, dim in enumerate(rtnmpc_minus_neural.y_reg_dims):
        #     plt.subplot(len(rtnmpc_minus_neural.y_reg_dims), 1, i + 1)
        #     # plt.plot(lin_acc_minus_neural[:, i], label="Acceleration by undisturbed model (minus)", color="tab:blue")
        #     # plt.plot(
        #     #     lin_acc_dist_minus_neural[:, i], label="Acceleration by disturbed model (minus)", color="tab:olive"
        #     # )
        #     # plt.plot(lin_acc_dist_minus_neural[:, i] - y[:, i], label="Neural Compensation (-)", color="tab:brown")
        #     plt.ylabel(f"D{dim}")
        #     if i == 0:
        #         plt.legend()
        #     plt.grid("on")
        #     plt.xlim(timestamp[0], timestamp[-1])
        #     if i != y.shape[1] - 1:
        #         ax = plt.gca()
        #         ax.axes.xaxis.set_ticklabels([])
        # mng = plt.get_current_fig_manager()
        # mng.resize(*mng.window.maxsize())
        # figures.append(fig)

        fig, _ = plt.subplots(figsize=(10, 5))
        plt.title("Model Output (plus)")
        for i, dim in enumerate(rtnmpc_plus_neural.y_reg_dims):
            plt.subplot(len(rtnmpc_plus_neural.y_reg_dims), 1, i + 1)
            plt.plot(lin_acc_plus_neural[:, i], label="Acceleration by undisturbed model (plus)", color="tab:blue")
            if rtnmpc_nominal.include_cog_dist_parameter:
                plt.plot(
                    lin_acc_dist_plus_neural[:, i], label="Acceleration by disturbed model (plus)", color="tab:olive"
                )
            # plt.plot(lin_acc_dist_plus_neural[:, i] + y[:, i], label="Neural Compensation (+)", color="tab:brown")
            plt.ylabel(f"D{dim}")
            if i == 0:
                plt.legend()
            plt.grid("on")
            plt.xlim(timestamp[0], timestamp[-1])
            if i != y.shape[1] - 1:
                ax = plt.gca()
                ax.axes.xaxis.set_ticklabels([])
        mng = plt.get_current_fig_manager()
        mng.resize(*mng.window.maxsize())
        figures.append(fig)

    #######################################################################
    fig, _ = plt.subplots(figsize=(10, 5))
    plt.title("Model Output")
    for i, dim in enumerate(rtnmpc_minus_neural.y_reg_dims):
        plt.subplot(len(rtnmpc_minus_neural.y_reg_dims), 1, i + 1)
        plt.plot(lin_acc_nominal[:, i], label="Acceleration by undisturbed model (nominal)", color="tab:blue")
        if rtnmpc_nominal.include_cog_dist_parameter:
            plt.plot(lin_acc_dist_nominal[:, i], label="Acceleration by disturbed model (nominal)", color="tab:olive")
            # plt.plot(lin_acc_minus_neural[:, i], label="Acceleration by undisturbed model (minus)", color="tab:green")
            # plt.plot(lin_acc_dist_minus_neural[:, i], label="Acceleration by disturbed model (minus)", color="tab:green")
            # plt.plot(lin_acc_dist_plus_neural[:, i], label="Acceleration by disturbed model (plus)", color="tab:red")
            plt.plot(lin_acc_dist_nominal[:, i] - y[:, i], label="Neural Compensation (-)", color="tab:orange")
            plt.plot(lin_acc_dist_nominal[:, i] + y[:, i], label="Neural Compensation (+)", color="tab:brown")
        plt.plot(lin_acc_plus_neural[:, i], label="Acceleration by undisturbed model (plus)", color="tab:red")
        plt.ylabel(f"D{dim}")
        if i == 0:
            plt.legend()
        plt.grid("on")
        plt.xlim(timestamp[0], timestamp[-1])
        if i != y.shape[1] - 1:
            ax = plt.gca()
            ax.axes.xaxis.set_ticklabels([])
    mng = plt.get_current_fig_manager()
    mng.resize(*mng.window.maxsize())
    figures.append(fig)

    for i, dim in enumerate(rtnmpc_minus_neural.y_reg_dims):
        plt.subplot(len(rtnmpc_minus_neural.y_reg_dims), 1, i + 1)
        if rtnmpc_nominal.include_cog_dist_parameter:
            plt.plot(lin_acc_dist_plus_neural[:, i], label="Acceleration by disturbed model (plus)", color="tab:olive")
        plt.plot(lin_acc_plus_neural[:, i], label="Acceleration by undisturbed model (plus)", color="tab:red")
        plt.ylabel(f"D{dim}")
        if i == 0:
            plt.legend()
        plt.grid("on")
        plt.xlim(timestamp[0], timestamp[-1])
        if i != y.shape[1] - 1:
            ax = plt.gca()
            ax.axes.xaxis.set_ticklabels([])
    mng = plt.get_current_fig_manager()
    mng.resize(*mng.window.maxsize())
    figures.append(fig)

    for i, dim in enumerate(rtnmpc_minus_neural.y_reg_dims):
        plt.subplot(len(rtnmpc_minus_neural.y_reg_dims), 1, i + 1)
        plt.plot(lin_acc_plus_neural[:, i], label="Acceleration by undisturbed model (plus)", color="tab:red")
        plt.ylabel(f"D{dim}")
        if i == 0:
            plt.legend()
        plt.grid("on")
        plt.xlim(timestamp[0], timestamp[-1])
        if i != y.shape[1] - 1:
            ax = plt.gca()
            ax.axes.xaxis.set_ticklabels([])
    mng = plt.get_current_fig_manager()
    mng.resize(*mng.window.maxsize())
    figures.append(fig)

    if save:
        save_dir = DirectoryConfig.SIMULATION_DIR
        safe_mkdir_recursive(save_dir)
        for i, fig in enumerate(figures):
            fig.savefig(os.path.join(save_dir, f"simulation_results_fig{i}.png"), dpi=500, bbox_inches="tight")
    halt = 1
