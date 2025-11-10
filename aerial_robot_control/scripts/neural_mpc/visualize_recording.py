import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

from utils.data_utils import undo_jsonify
from config.configurations import DirectoryConfig


def plot_trajectory():
    ###################################################
    # Load recording
    recfile = os.path.join(DirectoryConfig.DATA_DIR, "NMPCTiltQdServo_residual_dataset", "dataset_003.csv")
    # recfile = '/home/johannes/ros/raw_neural_mpc/neural-mpc/ros_dd_mpc/data/simplified_sim_dataset/train/dataset_001.csv'
    rec_dict = pd.read_csv(recfile)
    ###################################################

    state_in = undo_jsonify(rec_dict["state_in"].to_numpy())
    state_out = undo_jsonify(rec_dict["state_out"].to_numpy())
    state_prop = undo_jsonify(rec_dict["state_prop"].to_numpy())
    control = undo_jsonify(rec_dict["control"].to_numpy())
    # control = undo_jsonify(rec_dict['input_in'].to_numpy())
    timestamp = rec_dict["timestamp"].to_numpy()
    dt = rec_dict["dt"].to_numpy()

    # Plot state features
    fig = plt.subplots(figsize=(20, 5))
    n_plots = state_in.shape[1]  # , x.shape[1])
    for dim in range(state_in.shape[1] - 1, -1, -1):
        plt.subplot(n_plots, 2, dim * 2 + 1)
        plt.plot(timestamp, state_in[:, dim], label="state_in")
        plt.plot(timestamp, state_out[:, dim], label="state_out")
        plt.plot(timestamp, state_prop[:, dim], label="state_prop")
        plt.ylabel(f"D{dim}")
        if dim == 0:
            plt.title("State In & State Out")
            plt.legend()
        plt.grid("on")
        plt.xlim(timestamp[0], timestamp[-1])
        if dim != state_in.shape[1] - 1:
            ax = plt.gca()
            ax.axes.xaxis.set_ticklabels([])

    for dim in range(control.shape[1] - 1, -1, -1):
        plt.subplot(n_plots, 2, dim * 2 + 2)
        plt.plot(timestamp, control[:, dim], label="control")
        if dim == 0:
            plt.title("Control")
            plt.legend()
        plt.grid("on")
        plt.xlim(timestamp[0], timestamp[-1])
        if dim != state_in.shape[1] - 1:
            ax = plt.gca()
            ax.axes.xaxis.set_ticklabels([])

    fig = plt.figure(figsize=(20, 5))
    plt.plot(timestamp, state_in[:, 0], label="state_in")
    plt.plot(timestamp, state_out[:, 0], label="state_out")
    plt.plot(timestamp, state_prop[:, 0], label="state_prop")
    plt.grid("on")
    plt.title("Dim 0 zoom in")
    plt.legend()
    plt.xlim(timestamp[0], timestamp[-1])

    if "comp_time" in rec_dict.keys():
        fig = plt.figure(figsize=(20, 5))
        plt.plot(timestamp, rec_dict["comp_time"])
        plt.plot(
            [0, rec_dict["comp_time"].shape[0]],
            [np.mean(rec_dict["comp_time"]), np.mean(rec_dict["comp_time"])],
            color="r",
            label=f"Avg = {np.mean(rec_dict['comp_time']):.4f} ms",
        )
        plt.xlabel("Simulation time [s]")
        plt.ylabel("Computation time [ms]")
        plt.legend()
        plt.grid()
        plt.xlim(timestamp[0], timestamp[-1])

    dt = np.expand_dims(rec_dict["dt"], 1)

    diff1 = state_out - state_in
    d1 = diff1 / dt
    diff2 = state_prop - state_in
    d2 = diff2 / dt

    fig = plt.subplots(figsize=(20, 5))
    for dim in range(state_in.shape[1] - 1, -1, -1):
        plt.subplot(n_plots, 2, dim * 2 + 1)
        plt.plot(timestamp, d1[:, dim])
        plt.ylabel(f"D{dim}")
        if dim == 0:
            plt.title("(State Out - State In) / dt")
            plt.legend()
        plt.grid("on")
        plt.xlim(timestamp[0], timestamp[-1])
        if dim != state_in.shape[1] - 1:
            ax = plt.gca()
            ax.axes.xaxis.set_ticklabels([])

    for dim in range(state_in.shape[1] - 1, -1, -1):
        plt.subplot(n_plots, 2, dim * 2 + 2)
        plt.plot(timestamp, d2[:, dim], color="red")
        if dim == 0:
            plt.title("(State Pred - State In) / dt")
            plt.legend()
        plt.grid("on")
        plt.xlim(timestamp[0], timestamp[-1])
        if dim != state_in.shape[1] - 1:
            ax = plt.gca()
            ax.axes.xaxis.set_ticklabels([])
    # plt.show()

    diff3 = state_out - state_prop
    diff4 = state_out - state_prop
    d4 = diff4 / dt

    fig = plt.subplots(figsize=(20, 5))
    for dim in range(state_in.shape[1] - 1, -1, -1):
        plt.subplot(n_plots, 2, dim * 2 + 1)
        plt.plot(timestamp, diff3[:, dim])
        plt.ylabel(f"D{dim}")
        if dim == 0:
            plt.title("State Out - State Pred")
            plt.legend()
        plt.grid("on")
        plt.xlim(timestamp[0], timestamp[-1])
        if dim != state_in.shape[1] - 1:
            ax = plt.gca()
            ax.axes.xaxis.set_ticklabels([])

    for dim in range(state_in.shape[1] - 1, -1, -1):
        plt.subplot(n_plots, 2, dim * 2 + 2)
        plt.plot(timestamp, d4[:, dim], color="red")
        if dim == 0:
            plt.title("(State Out - State Pred) / dt")
            plt.legend()
        plt.grid("on")
        plt.xlim(timestamp[0], timestamp[-1])
        if dim != state_in.shape[1] - 1:
            ax = plt.gca()
            ax.axes.xaxis.set_ticklabels([])
    plt.show()
    halt = 1


if __name__ == "__main__":
    plot_trajectory()
