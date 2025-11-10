import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

from config.configurations import DirectoryConfig
from utils.data_utils import undo_jsonify
from utils.moving_average_filter import moving_average_filter


def plot_trajectory():
    ###################################################
    # Load recording
    recfile = (
        # "NMPCTiltQdServo_residual_dataset_04", "dataset_001.csv"
        # "NMPCTiltQdServo_residual_dataset_06", "dataset_001.csv"
        # "NMPCTiltQdServo_real_machine_dataset_GROUND_EFFECT_ONLY", "dataset_002.csv"
        "NMPCTiltQdServo_real_machine_dataset_TRAIN_FOR_PAPER",
        "dataset_001.csv"
        # "NMPCTiltQdServo_real_machine_dataset_VAL_FOR_PAPER", "dataset_003.csv"
        # "NMPCTiltQdServo_residual_dataset_neural_sim_nominal_control_07", "dataset_001.csv"
    )
    ###################################################
    print(f"Loading recording from {recfile}")

    recfile = os.path.join(DirectoryConfig.DATA_DIR, *recfile)
    rec_dict = pd.read_csv(recfile)
    state_in = undo_jsonify(rec_dict["state_in"].to_numpy())
    state_out = undo_jsonify(rec_dict["state_out"].to_numpy())
    state_prop = undo_jsonify(rec_dict["state_prop"].to_numpy())
    control = undo_jsonify(rec_dict["control"].to_numpy())
    timestamp = rec_dict["timestamp"].to_numpy()
    dt = rec_dict["dt"].to_numpy()

    # State features
    plt.subplots(figsize=(20, 5))
    for dim in range(state_in.shape[1] - 1, -1, -1):
        plt.subplot(state_in.shape[1], 1, dim + 1)
        plt.plot(timestamp, state_in[:, dim], label="state_in")
        plt.plot(timestamp, state_out[:, dim], label="state_out")
        plt.plot(timestamp, state_prop[:, dim], label="state_prop")
        plt.xlim(timestamp[0], timestamp[-1])
        plt.ylabel(f"D{dim}")
        plt.grid("on")
        if dim == 0:
            plt.title("State In & State Out")
            plt.legend(loc="upper right")
        if dim != state_in.shape[1] - 1:
            ax = plt.gca()
            ax.axes.xaxis.set_ticklabels([])

    # Control features
    plt.subplots(figsize=(20, 5))
    labels = ["T1 [N]", "T2 [N]", "T3 [N]", "T4 [N]", "S1 [rad]", "S2 [rad]", "S3 [rad]", "S4 [rad]"]
    for dim in range(control.shape[1] - 1, -1, -1):
        plt.subplot(control.shape[1], 1, dim + 1)
        plt.plot(timestamp, control[:, dim])
        plt.xlim(timestamp[0], timestamp[-1])
        plt.ylabel(labels[dim])
        plt.grid("on")
        if dim == 0:
            plt.title("Control")
        if dim != state_in.shape[1] - 1:
            ax = plt.gca()
            ax.axes.xaxis.set_ticklabels([])

    # Zoom in on vz
    plt.figure(figsize=(20, 5))
    plt.title("Dim 5 zoom in")
    plt.plot(timestamp, state_in[:, 5], label="state_in")
    plt.plot(timestamp, state_out[:, 5], label="state_out")
    plt.plot(timestamp, state_prop[:, 5], label="state_prop")
    plt.xlim(timestamp[0], timestamp[-1])
    plt.grid("on")
    plt.legend(loc="upper right")

    # Computation time
    if "comp_time" in rec_dict.keys():
        plt.figure(figsize=(20, 5))
        plt.plot(timestamp, rec_dict["comp_time"])
        plt.plot(
            [0, rec_dict["comp_time"].shape[0]],
            [np.mean(rec_dict["comp_time"]), np.mean(rec_dict["comp_time"])],
            color="r",
            label=f"Avg = {np.mean(rec_dict['comp_time']):.4f} ms",
        )
        plt.xlim(timestamp[0], timestamp[-1])
        plt.xlabel("Simulation time [s]")
        plt.ylabel("Computation time [ms]")
        plt.legend(loc="upper right")
        plt.grid("on")

    # Labels
    dt = np.expand_dims(rec_dict["dt"], 1)
    y_true = (state_out - state_prop) / dt
    a_idx = [3, 4, 5]

    # Apply moving average filter to smooth the labels
    # y_true_filtered = moving_average_filter(y_true, window_size=5)

    # Labels zoom in on acceleration
    plt.subplots(figsize=(20, 5))
    for i, dim in enumerate(a_idx):
        plt.subplot(len(a_idx), 1, i + 1)
        plt.plot(timestamp, y_true[:, dim], label="Raw")
        # plt.plot(timestamp, y_true_filtered[:, dim], label="Filtered", color="red", linewidth=2)
        plt.xlim(timestamp[0], timestamp[-1])
        plt.ylabel(f"D{dim}")
        plt.grid("on")
        if i == 0:
            plt.title("Labels [(State Out - State Prop) / dt] - Zoom in on ax, ay, az")
            plt.legend(loc="upper right")
        if i != state_in.shape[1] - 1:
            ax = plt.gca()
            ax.axes.xaxis.set_ticklabels([])

    # Labels zoom in on az
    plt.figure(figsize=(20, 5))
    plt.title("Labels [(State Out - State Prop) / dt] - Dim 5 zoom in")
    plt.plot(timestamp, y_true[:, 5], label="Raw")
    # plt.plot(timestamp, y_true_filtered[:, 5], label="Filtered", color="red", linewidth=2)
    plt.xlim(timestamp[0], timestamp[-1])
    plt.legend(loc="upper right")
    plt.grid("on")

    plt.show()
    halt = 1


if __name__ == "__main__":
    plot_trajectory()
