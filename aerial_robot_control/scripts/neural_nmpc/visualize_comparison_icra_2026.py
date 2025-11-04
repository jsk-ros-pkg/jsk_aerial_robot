import numpy as np
import torch
import os, sys
import matplotlib.pyplot as plt
import scienceplots  # DONT DELETE!
from mpl_toolkits.mplot3d import Axes3D  # DONT DELETE!

from utils.geometry_utils import v_dot_q, quaternion_inverse
from config.configurations import DirectoryConfig, EnvConfig, ModelFitConfig
from utils.data_utils import safe_mkdir_recursive, undo_jsonify, read_dataset
from sim_environment.forward_prop import init_forward_prop
from utils.model_utils import load_model
from neural_controller_standalone import NeuralNMPC

sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from nmpc.nmpc_tilt_mt.tilt_qd import phys_param_beetle_omni as phys_omni

matlab_blue = "#0072BD"
matlab_orange = "#D95319"
matlab_yellow = "#EDB120"
matlab_green = "#77AC30"
matlab_purple = "#7E2F8E"


def plot_comparison(rec_dict, rtnmpc_neural_ctrl):
    """
    Plot comparison between nominal and neural NMPC simulation results.

    Args:
        model_options (dict): Dictionary containing model configuration options.
        rec_dict (dict): Dictionary containing recorded simulation data.
    """
    # Extract recorded data
    timestamp = rec_dict["timestamp"]
    target_nominal = rec_dict["target_nominal"]
    target_neural = rec_dict["target_neural"]
    state_in_neural = rec_dict["state_in_neural"]
    state_in_nominal = rec_dict["state_in_nominal"]
    state_out_neural = rec_dict["state_out_neural"]
    state_out_nominal = rec_dict["state_out_nominal"]
    state_prop = rec_dict["state_prop"]
    control_neural = rec_dict["control_neural"]
    control_nominal = rec_dict["control_nominal"]
    comp_time_neural = rec_dict["comp_time_neural"]
    comp_time_nominal = rec_dict["comp_time_nominal"]

    # Plot settings
    plt.style.use(["science", "grid"])
    plt.rcParams.update({"font.size": 11})
    label_size = 14
    figsize = (7, 6)
    linewidth = 2
    alpha = 0.2

    time_start_rotate1 = 5.5
    time_stop_rotate1 = 9
    time_start_rotate2 = 15
    time_stop_rotate2 = 18.5
    time_start_rotate3 = 24
    time_stop_rotate3 = 28

    idx_start = 0
    idx_end = len(timestamp)

    # --- Position
    plt.subplots(figsize=figsize)
    plt.subplot(3, 1, 1)
    plt.plot(
        timestamp[idx_start:idx_end],
        target_nominal[idx_start:idx_end, 0],
        label=r"Nominal Reference",
        color="tab:orange",
        linestyle="--",
        alpha=0.5,
    )
    plt.plot(
        timestamp[idx_start:idx_end],
        target_neural[idx_start:idx_end, 0],
        label=r"Neural Reference",
        color="tab:red",
        linestyle="--",
        alpha=0.5,
    )
    plt.plot(timestamp[idx_start:idx_end], state_out_nominal[idx_start:idx_end, 0], label=r"Nominal MPC")
    # plt.plot(timestamp[idx_start:idx_end], state_prop[idx_start:idx_end, 0], label=r"Nominal MPC")
    plt.plot(
        timestamp[idx_start:idx_end], state_out_neural[idx_start:idx_end, 0], label=r"Neural MPC", color=matlab_yellow
    )
    plt.xlim(timestamp[idx_start], timestamp[idx_end - 1])
    plt.ylabel("$x$ [m]", fontsize=label_size)
    plt.legend(loc="upper right", ncol=3, fancybox=True, framealpha=0.5, fontsize=label_size - 1)
    plt.grid("on")
    ax = plt.gca()
    ax.axes.xaxis.set_ticklabels([])

    plt.subplot(3, 1, 2)
    plt.plot(
        timestamp[idx_start:idx_end],
        target_nominal[idx_start:idx_end, 1],
        label=r"Nominal Reference",
        color="tab:orange",
        linestyle="--",
        alpha=0.5,
    )
    plt.plot(
        timestamp[idx_start:idx_end],
        target_neural[idx_start:idx_end, 1],
        label=r"Neural Reference",
        color="tab:red",
        linestyle="--",
        alpha=0.5,
    )
    plt.plot(timestamp[idx_start:idx_end], state_out_nominal[idx_start:idx_end, 1])
    # plt.plot(timestamp[idx_start:idx_end], state_prop[idx_start:idx_end, 1])
    plt.plot(timestamp[idx_start:idx_end], state_out_neural[idx_start:idx_end, 1], color=matlab_yellow)
    plt.xlim(timestamp[idx_start], timestamp[idx_end - 1])
    plt.ylabel("$y$ [m]", fontsize=label_size)
    plt.grid("on")
    ax = plt.gca()
    ax.axes.xaxis.set_ticklabels([])

    plt.subplot(3, 1, 3)
    plt.plot(
        timestamp[idx_start:idx_end],
        target_nominal[idx_start:idx_end, 2],
        label=r"Nominal Reference",
        color="tab:orange",
        linestyle="--",
        alpha=0.5,
    )
    plt.plot(
        timestamp[idx_start:idx_end],
        target_neural[idx_start:idx_end, 2],
        label=r"Neural Reference",
        color="tab:red",
        linestyle="--",
        alpha=0.5,
    )
    plt.plot(timestamp[idx_start:idx_end], state_out_nominal[idx_start:idx_end, 2])
    # plt.plot(timestamp[idx_start:idx_end], state_prop[idx_start:idx_end, 2])
    plt.plot(timestamp[idx_start:idx_end], state_out_neural[idx_start:idx_end, 2], color=matlab_yellow)
    plt.xlim(timestamp[idx_start], timestamp[idx_end - 1])
    plt.xlabel("$t$ [s]", fontsize=label_size)
    plt.ylabel("$z$ [m]", fontsize=label_size)
    plt.grid("on")

    # --- Velocity
    linewidth = 2
    plt.subplots(figsize=(7, 9))
    ax = plt.subplot(3, 1, 1)
    ax2 = ax.twinx()
    ax.axvspan(time_start_rotate1, time_stop_rotate1, facecolor=matlab_yellow, alpha=alpha)
    # ax.axvspan(time_start_rotate2, time_stop_rotate2, facecolor=matlab_yellow, alpha=alpha)
    ax.axvspan(time_start_rotate3, time_stop_rotate3, facecolor=matlab_yellow, alpha=alpha)
    # ax2.plot(timestamp[idx_start:idx_end], state_out_neural[idx_start:idx_end, 3] - state_prop[idx_start:idx_end, 3], label="Difference", color="r", linewidth=linewidth, linestyle="--", alpha=0.5)
    # ax2.plot(timestamp[idx_start:idx_end], state_out_neural[idx_start:idx_end, 3] - state_out_nominal[idx_start:idx_end, 3], label="Difference", color="r", linewidth=linewidth, linestyle="--", alpha=0.5)
    # ax.plot(timestamp[idx_start:idx_end], state_prop[idx_start:idx_end, 3], label=r"Nominal MPC", linewidth=linewidth, color="tab:blue")#, linestyle="--")
    ax.plot(
        timestamp[idx_start:idx_end], state_out_nominal[idx_start:idx_end, 3], label=r"Nominal MPC", linewidth=linewidth
    )  # , linestyle="--", alpha=0.5)
    ax.plot(
        timestamp[idx_start:idx_end],
        state_out_neural[idx_start:idx_end, 3],
        label=r"Neural MPC",
        color=matlab_yellow,
        linewidth=linewidth,
    )  # , linestyle="--")
    plt.xlim(timestamp[idx_start], timestamp[idx_end - 1])
    ax2.set_ylim([-0.5, 0.5])
    ax.set_ylabel("$v_x$ [m/s]", fontsize=label_size)
    lines, labels = ax.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax2.legend(
        lines + lines2,
        labels + labels2,
        loc="upper right",
        ncol=3,
        fancybox=True,
        framealpha=0.5,
        fontsize=label_size - 1,
    )
    plt.grid("on")
    ax = plt.gca()
    ax.axes.xaxis.set_ticklabels([])

    ax = plt.subplot(3, 1, 2)
    ax2 = ax.twinx()
    ax.axvspan(time_start_rotate1, time_stop_rotate1, facecolor=matlab_yellow, alpha=alpha)
    # ax.axvspan(time_start_rotate2, time_stop_rotate2, facecolor=matlab_yellow, alpha=alpha)
    ax.axvspan(time_start_rotate3, time_stop_rotate3, facecolor=matlab_yellow, alpha=alpha)
    # ax2.plot(timestamp[idx_start:idx_end], state_out_neural[idx_start:idx_end, 4] - state_prop[idx_start:idx_end, 4], color="r", linewidth=linewidth, linestyle="--", alpha=0.5)
    # ax2.plot(timestamp[idx_start:idx_end], state_out_neural[idx_start:idx_end, 4] - state_out_nominal[idx_start:idx_end, 4], color="r", linewidth=linewidth, linestyle="--", alpha=0.5)
    # ax.plot(timestamp[idx_start:idx_end], state_prop[idx_start:idx_end, 4], linewidth=linewidth, color="tab:blue")#, linestyle="--")
    ax.plot(
        timestamp[idx_start:idx_end], state_out_nominal[idx_start:idx_end, 4], linewidth=linewidth
    )  # , linestyle="--", alpha=0.5)
    ax.plot(
        timestamp[idx_start:idx_end], state_out_neural[idx_start:idx_end, 4], color=matlab_yellow, linewidth=linewidth
    )  # , linestyle="--")
    plt.xlim(timestamp[idx_start], timestamp[idx_end - 1])
    ax2.set_ylim([-0.5, 0.5])
    ax.set_ylabel("$v_y$ [m/s]", fontsize=label_size)
    plt.grid("on")
    ax = plt.gca()
    ax.axes.xaxis.set_ticklabels([])

    ax = plt.subplot(3, 1, 3)
    ax2 = ax.twinx()
    ax.axvspan(time_start_rotate1, time_stop_rotate1, facecolor=matlab_yellow, alpha=alpha)
    # ax.axvspan(time_start_rotate2, time_stop_rotate2, facecolor=matlab_yellow, alpha=alpha)
    ax.axvspan(time_start_rotate3, time_stop_rotate3, facecolor=matlab_yellow, alpha=alpha)
    # ax2.plot(timestamp[idx_start:idx_end], state_out_neural[idx_start:idx_end, 5] - state_prop[idx_start:idx_end, 5], color="r", linewidth=linewidth, linestyle="--", alpha=0.5)
    # ax2.plot(timestamp[idx_start:idx_end], state_out_neural[idx_start:idx_end, 5] - state_out_nominal[idx_start:idx_end, 5], color="r", linewidth=linewidth, linestyle="--", alpha=0.5)
    # ax.plot(timestamp[idx_start:idx_end], state_prop[idx_start:idx_end, 5], linewidth=linewidth, color="tab:blue")#, linestyle="--")
    ax.plot(
        timestamp[idx_start:idx_end], state_out_nominal[idx_start:idx_end, 5], linewidth=linewidth
    )  # , linestyle="--", alpha=0.5)
    ax.plot(
        timestamp[idx_start:idx_end], state_out_neural[idx_start:idx_end, 5], color=matlab_yellow, linewidth=linewidth
    )  # , linestyle="--")
    ax.set_xlim(timestamp[idx_start], timestamp[idx_end - 1])
    ax.set_xlabel("$t$ [s]", fontsize=label_size)
    ax2.set_ylim([-0.5, 0.5])
    ax.set_ylabel("$v_z$ [m/s]", fontsize=label_size)
    plt.grid("on")

    # --- Thrust Cmds
    plt.subplots(figsize=figsize)
    plt.subplot(4, 1, 1)
    plt.plot(timestamp[idx_start:idx_end], control_neural[idx_start:idx_end, 0], label="Nominal Cmd")
    plt.plot(timestamp[idx_start:idx_end], control_nominal[idx_start:idx_end, 0], label="Neural Cmd")
    plt.xlim(timestamp[idx_start], timestamp[idx_end - 1])
    plt.ylabel("Rotor 0 [N]", fontsize=label_size)
    plt.legend(loc="upper left", ncol=4, fancybox=True, framealpha=0.5, fontsize=label_size - 2)
    plt.grid("on")
    ax = plt.gca()
    ax.axes.xaxis.set_ticklabels([])

    plt.subplot(4, 1, 2)
    plt.plot(timestamp[idx_start:idx_end], control_neural[idx_start:idx_end, 1])
    plt.plot(timestamp[idx_start:idx_end], control_nominal[idx_start:idx_end, 1])
    plt.xlim(timestamp[idx_start], timestamp[idx_end - 1])
    plt.ylabel("Rotor 1 [N]", fontsize=label_size)
    plt.grid("on")
    ax = plt.gca()
    ax.axes.xaxis.set_ticklabels([])

    plt.subplot(4, 1, 3)
    plt.plot(timestamp[idx_start:idx_end], control_neural[idx_start:idx_end, 2])
    plt.plot(timestamp[idx_start:idx_end], control_nominal[idx_start:idx_end, 2])
    plt.xlim(timestamp[idx_start], timestamp[idx_end - 1])
    plt.ylabel("Rotor 2 [N]", fontsize=label_size)
    plt.grid("on")
    ax = plt.gca()
    ax.axes.xaxis.set_ticklabels([])

    plt.subplot(4, 1, 4)
    plt.plot(timestamp[idx_start:idx_end], control_neural[idx_start:idx_end, 3])
    plt.plot(timestamp[idx_start:idx_end], control_nominal[idx_start:idx_end, 3])
    plt.xlim(timestamp[idx_start], timestamp[idx_end - 1])
    plt.xlabel("$t$ [s]", fontsize=label_size)
    plt.ylabel("Rotor 3 [N]", fontsize=label_size)
    plt.grid("on")

    # --- Servo Angles
    plt.subplots(figsize=figsize)
    plt.subplot(4, 1, 1)
    plt.plot(timestamp[idx_start:idx_end], control_neural[idx_start:idx_end, 4], label="Nominal Cmd")
    plt.plot(timestamp[idx_start:idx_end], control_nominal[idx_start:idx_end, 4], label="Neural Cmd")
    plt.xlim(timestamp[idx_start], timestamp[idx_end - 1])
    plt.ylabel("Servo angle 0 [rad]", fontsize=label_size)
    plt.legend(loc="upper left", ncol=4, fancybox=True, framealpha=0.5, fontsize=label_size - 2)
    plt.grid("on")
    ax = plt.gca()
    ax.axes.xaxis.set_ticklabels([])

    plt.subplot(4, 1, 2)
    plt.plot(timestamp[idx_start:idx_end], control_neural[idx_start:idx_end, 5])
    plt.plot(timestamp[idx_start:idx_end], control_nominal[idx_start:idx_end, 5])
    plt.xlim(timestamp[idx_start], timestamp[idx_end - 1])
    plt.ylabel("Servo angle 1 [rad]", fontsize=label_size)
    plt.grid("on")
    ax = plt.gca()
    ax.axes.xaxis.set_ticklabels([])

    plt.subplot(4, 1, 3)
    plt.plot(timestamp[idx_start:idx_end], control_neural[idx_start:idx_end, 6])
    plt.plot(timestamp[idx_start:idx_end], control_nominal[idx_start:idx_end, 6])
    plt.xlim(timestamp[idx_start], timestamp[idx_end - 1])
    plt.ylabel("Servo angle 2 [rad]", fontsize=label_size)
    plt.grid("on")
    ax = plt.gca()
    ax.axes.xaxis.set_ticklabels([])

    plt.subplot(4, 1, 4)
    plt.plot(timestamp[idx_start:idx_end], control_neural[idx_start:idx_end, 7])
    plt.plot(timestamp[idx_start:idx_end], control_nominal[idx_start:idx_end, 7])
    plt.xlim(timestamp[idx_start], timestamp[idx_end - 1])
    plt.xlabel("$t$ [s]", fontsize=label_size)
    plt.ylabel("Servo angle 3 [rad]", fontsize=label_size)
    plt.grid("on")

    # --- Computation Time
    # plt.figure(figsize=(7, 2))
    # plt.plot(timestamp[idx_start:idx_end], comp_time_nominal[idx_start:idx_end], label=r"Nominal MPC", color="tab:green", alpha=0.5)
    # plt.plot([timestamp[idx_start], timestamp[idx_end-1]], [np.mean(comp_time_nominal[idx_start:idx_end]), np.mean(comp_time_nominal[idx_start:idx_end])],
    #             label=r"Mean - Nominal MPC", color="tab:red", linestyle="--"
    # )
    # plt.plot(timestamp[idx_start:idx_end], comp_time_neural[idx_start:idx_end], label=r"Neural MPC", color="tab:olive", alpha=0.5)
    # plt.plot([timestamp[idx_start], timestamp[idx_end-1]], [np.mean(comp_time_neural[idx_start:idx_end]), np.mean(comp_time_neural[idx_start:idx_end])],
    #             label=r"Mean - Neural MPC", color="red", linestyle="--"
    # )
    # plt.xlim(timestamp[idx_start], timestamp[idx_end-1])
    # plt.xlabel("$t$ [ms]", fontsize=label_size)
    # plt.ylabel("Comp. time [s]", fontsize=label_size)
    # plt.legend(loc="upper left", ncol=2, fancybox=True, framealpha=0.5, fontsize=label_size - 1)
    # plt.grid("on")

    # --- Model Output
    def velocity_mapping(state_sequence):
        p_traj = state_sequence[:, :3]
        v_w_traj = state_sequence[:, 3:6]
        q_traj = state_sequence[:, 6:10]
        other_traj = state_sequence[:, 10:]  # w, a_s, f_s, etc.

        v_b_traj = np.empty_like(v_w_traj)
        for t in range(len(v_w_traj)):
            v_b_traj[t, :] = v_dot_q(v_w_traj[t, :], quaternion_inverse(q_traj[t, :]))
        return np.concatenate((p_traj, v_b_traj, q_traj, other_traj), axis=1)

    if rtnmpc_neural_ctrl.mlp_metadata["ModelFitConfig"]["input_transform"]:
        state_in_mlp = velocity_mapping(state_in_nominal)
    else:
        # Don't transform input but let network learn in world frame directly
        state_in_mlp = state_in_nominal.copy()

    state_feats = eval(rtnmpc_neural_ctrl.mlp_metadata["ModelFitConfig"]["state_feats"])
    u_feats = eval(rtnmpc_neural_ctrl.mlp_metadata["ModelFitConfig"]["u_feats"])
    device = "cpu"
    state_in_mlp_tensor = torch.from_numpy(state_in_mlp).type(torch.float32).to(torch.device(device))
    control_tensor = torch.from_numpy(control_neural).type(torch.float32).to(torch.device(device))
    mlp_in = torch.cat((state_in_mlp_tensor[:, state_feats], control_tensor[:, u_feats]), axis=1)

    mlp_out = rtnmpc_neural_ctrl.neural_model(mlp_in)
    mlp_out = mlp_out.detach().cpu().numpy()

    if rtnmpc_neural_ctrl.mlp_metadata["ModelFitConfig"]["label_transform"]:
        for t in range(state_in_nominal.shape[0]):
            mlp_out[t, :] = v_dot_q(mlp_out[t, :], state_in_nominal[t, 6:10])

    plt.subplots(figsize=figsize)

    plt.subplot(3, 1, 1)
    plt.plot(
        timestamp[idx_start:idx_end], mlp_out[idx_start:idx_end, 0], color=matlab_yellow
    )  # , label=r"$\boldsymbol{f}_\mathrm{Neural}$")
    plt.xlim(timestamp[idx_start], timestamp[idx_end - 1])
    plt.ylabel(r"$\tilde{a}_x$ [m/s$^2$]", fontsize=label_size)
    plt.grid("on")
    ax = plt.gca()
    ax.axes.xaxis.set_ticklabels([])

    plt.subplot(3, 1, 2)
    plt.plot(timestamp[idx_start:idx_end], mlp_out[idx_start:idx_end, 1], color=matlab_yellow)
    plt.xlim(timestamp[idx_start], timestamp[idx_end - 1])
    plt.ylabel(r"$\tilde{a}_y$ [m/s$^2$]", fontsize=label_size)
    plt.grid("on")
    ax = plt.gca()
    ax.axes.xaxis.set_ticklabels([])

    plt.subplot(3, 1, 3)
    plt.plot(timestamp[idx_start:idx_end], mlp_out[idx_start:idx_end, 2], color=matlab_yellow)
    plt.xlim(timestamp[idx_start], timestamp[idx_end - 1])
    plt.xlabel("$t$ [s]", fontsize=label_size)
    plt.ylabel(r"$\tilde{a}_z$ [m/s$^2$]", fontsize=label_size)
    plt.grid("on")

    # ================ Compensation ================
    class struct:
        def __init__(self):
            pass

    nmpc = struct()
    nmpc.tilt = True
    nmpc.include_servo_model = True
    nmpc.include_thrust_model = False
    nmpc.include_servo_derivative = False
    nmpc.include_cog_dist_parameter = True
    nmpc.phys = struct()

    nmpc.phys = phys_omni

    # Define nominal model
    dynamics, _, _ = init_forward_prop(nmpc)

    # Compute linear acceleration with nominal model
    x_dot = np.empty(state_in_nominal.shape)
    for t in range(state_in_nominal.shape[0]):
        x_dot[t, :] = np.array(dynamics(x=state_in_nominal[t, :], u=control_neural[t, :])["x_dot"]).squeeze()
    lin_acc = x_dot[:, 3:6]

    # Plot simulation results for acceleration and the effect of the neural model
    # idx_start, idx_end = 19000, 19501
    # figsize = (14, 12)
    plt.subplots(figsize=(14, 6))
    ax = plt.subplot(3, 1, 1)
    ax.axvspan(time_start_rotate1, time_stop_rotate1, facecolor=matlab_yellow, alpha=alpha)
    # ax.axvspan(time_start_rotate2, time_stop_rotate2, facecolor=matlab_yellow, alpha=alpha)
    ax.axvspan(time_start_rotate3, time_stop_rotate3, facecolor=matlab_yellow, alpha=alpha)
    ax.plot(
        timestamp[idx_start:idx_end],
        lin_acc[idx_start:idx_end, 0],
        label=r"Nominal model $\boldsymbol{a}$",
        linewidth=linewidth,
    )
    ax.plot(
        timestamp[idx_start:idx_end],
        lin_acc[idx_start:idx_end, 0] + mlp_out[idx_start:idx_end, 0],
        label=r"Neural Compensation $\boldsymbol{a} + \boldsymbol{f}_{\mathrm{Neural}}$",
        color=matlab_orange,
        linewidth=linewidth,
        linestyle="--",
    )
    # ln3 = ax_err.fill_between(t, 0, abs_err, color=matlab_blue, alpha=0.2, label="error")
    ax.set_xlim(timestamp[idx_start], timestamp[idx_end - 1])
    ax.set_ylabel("$a_x$ [m/s$^2$]", fontsize=label_size)
    ax.legend(loc="upper right", ncol=2, fancybox=True, framealpha=0.5, fontsize=label_size - 2)
    ax.grid("on")
    ax.axes.xaxis.set_ticklabels([])

    ax = plt.subplot(3, 1, 2)
    ax.axvspan(time_start_rotate1, time_stop_rotate1, facecolor=matlab_yellow, alpha=alpha)
    # ax.axvspan(time_start_rotate2, time_stop_rotate2, facecolor=matlab_yellow, alpha=alpha)
    ax.axvspan(time_start_rotate3, time_stop_rotate3, facecolor=matlab_yellow, alpha=alpha)
    ax.plot(timestamp[idx_start:idx_end], lin_acc[idx_start:idx_end, 1], linewidth=linewidth)
    ax.plot(
        timestamp[idx_start:idx_end],
        lin_acc[idx_start:idx_end, 1] + mlp_out[idx_start:idx_end, 1],
        color=matlab_orange,
        linewidth=linewidth,
        linestyle="--",
    )
    ax.set_xlim(timestamp[idx_start], timestamp[idx_end - 1])
    ax.set_ylabel("$a_y$ [m/s$^2$]", fontsize=label_size)
    ax.grid("on")
    ax.axes.xaxis.set_ticklabels([])

    ax = plt.subplot(3, 1, 3)
    ax.axvspan(time_start_rotate1, time_stop_rotate1, facecolor=matlab_yellow, alpha=alpha)
    # ax.axvspan(time_start_rotate2, time_stop_rotate2, facecolor=matlab_yellow, alpha=alpha)
    ax.axvspan(time_start_rotate3, time_stop_rotate3, facecolor=matlab_yellow, alpha=alpha)
    ax.plot(timestamp[idx_start:idx_end], lin_acc[idx_start:idx_end, 2], linewidth=linewidth)
    ax.plot(
        timestamp[idx_start:idx_end],
        lin_acc[idx_start:idx_end, 2] + mlp_out[idx_start:idx_end, 2],
        color=matlab_orange,
        linewidth=linewidth,
        linestyle="--",
    )
    ax.set_xlim(timestamp[idx_start], timestamp[idx_end - 1])
    ax.set_xlabel("$t$ [s]", fontsize=label_size)
    ax.set_ylabel("$a_z$ [m/s$^2$]", fontsize=label_size)
    ax.grid("on")

    # --- Ground effect
    plt.subplots(figsize=(7, 6))
    ax = plt.subplot(3, 1, 1)
    ax.axvspan(time_start_rotate1, time_stop_rotate1, facecolor=matlab_yellow, alpha=alpha)
    # ax.axvspan(time_start_rotate2, time_stop_rotate2, facecolor=matlab_yellow, alpha=alpha)
    ax.axvspan(time_start_rotate3, time_stop_rotate3, facecolor=matlab_yellow, alpha=alpha)
    ax.plot(timestamp[idx_start:idx_end], control_neural[idx_start:idx_end, 0], color=matlab_green)
    ax.plot(timestamp[idx_start:idx_end], control_neural[idx_start:idx_end, 1], color=matlab_green)
    ax.plot(timestamp[idx_start:idx_end], control_neural[idx_start:idx_end, 2], color=matlab_green)
    ax.plot(timestamp[idx_start:idx_end], control_neural[idx_start:idx_end, 3], color=matlab_green)
    ax.set_xlim(timestamp[idx_start], timestamp[idx_end - 1])
    ax.set_ylabel("Thrust cmd [N]", fontsize=label_size)
    ax.axes.xaxis.set_ticklabels([])
    ax.grid("on")

    ax = plt.subplot(3, 1, 2)
    ax.axvspan(time_start_rotate1, time_stop_rotate1, facecolor=matlab_yellow, alpha=alpha)
    ax.axvspan(time_start_rotate2, time_stop_rotate2, facecolor=matlab_yellow, alpha=alpha)
    ax.axvspan(time_start_rotate3, time_stop_rotate3, facecolor=matlab_yellow, alpha=alpha)
    ax.plot(timestamp[idx_start:idx_end], state_in_neural[idx_start:idx_end, 2], color=matlab_blue)
    ax.set_xlim(timestamp[idx_start], timestamp[idx_end - 1])
    ax.set_ylabel("$z$ [m]", fontsize=label_size)
    ax.axes.xaxis.set_ticklabels([])
    ax.grid("on")

    ax = plt.subplot(3, 1, 3)
    ax.axvspan(time_start_rotate1, time_stop_rotate1, facecolor=matlab_yellow, alpha=alpha)
    # ax.axvspan(time_start_rotate2, time_stop_rotate2, facecolor=matlab_yellow, alpha=alpha)
    ax.axvspan(time_start_rotate3, time_stop_rotate3, facecolor=matlab_yellow, alpha=alpha)
    ax.plot(timestamp[idx_start:idx_end], lin_acc[idx_start:idx_end, 2], linewidth=linewidth)
    ax.plot(
        timestamp[idx_start:idx_end],
        lin_acc[idx_start:idx_end, 2] + mlp_out[idx_start:idx_end, 2],
        color=matlab_orange,
        linewidth=linewidth,
        linestyle="--",
    )
    ax.set_xlim(timestamp[idx_start], timestamp[idx_end - 1])
    ax.set_xlabel("$t$ [s]", fontsize=label_size)
    ax.set_ylabel("$a_z$ [m/s$^2$]", fontsize=label_size)
    ax.grid("on")
    halt = 1


if __name__ == "__main__":
    # Load nominal controller simulation
    ds_name_nominal = "NMPCTiltQdServo_compare_nominal_neural_sim_04"  # 03
    ds_instance_nominal = "dataset_001"
    df_nominal = read_dataset(ds_name_nominal, ds_instance_nominal)

    target_nominal = undo_jsonify(df_nominal["target"].to_numpy())
    state_in_nominal = undo_jsonify(df_nominal["state_in"].to_numpy())
    state_out_nominal = undo_jsonify(df_nominal["state_out"].to_numpy())
    state_prop_nominal = undo_jsonify(df_nominal["state_prop"].to_numpy())
    control_nominal = undo_jsonify(df_nominal["control"].to_numpy())

    # Load neural controller simulation
    ds_name_neural = "NMPCTiltQdServo_compare_nominal_neural_sim_05"  # 06"#02"
    ds_instance_neural = "dataset_001"
    df_neural = read_dataset(ds_name_neural, ds_instance_neural)

    target_neural = undo_jsonify(df_neural["target"].to_numpy())
    state_in_neural = undo_jsonify(df_neural["state_in"].to_numpy())
    state_out_neural = undo_jsonify(df_neural["state_out"].to_numpy())
    state_prop_neural = undo_jsonify(df_neural["state_prop"].to_numpy())
    control_neural = undo_jsonify(df_neural["control"].to_numpy())

    rec_dict = {
        "timestamp": df_neural["timestamp"].to_numpy(),
        "target_neural": target_neural,
        "target_nominal": target_nominal,
        "state_in_neural": state_in_neural,
        "state_in_nominal": state_in_nominal,
        "state_out_neural": state_out_neural,
        "state_out_nominal": state_out_nominal,
        "state_prop": state_prop_neural,
        "control_neural": control_neural,
        "control_nominal": control_nominal,
        "comp_time_neural": df_neural["comp_time"].to_numpy(),
        "comp_time_nominal": df_nominal["comp_time"].to_numpy(),
    }

    # Plot
    model_options = EnvConfig.model_options
    model_options["only_use_nominal"] = False
    rtnmpc_neural_ctrl = NeuralNMPC(
        model_options=model_options,
        solver_options=EnvConfig.solver_options,
        sim_options=EnvConfig.sim_options,
        run_options=EnvConfig.run_options,
    )
    plot_comparison(rec_dict, rtnmpc_neural_ctrl)
