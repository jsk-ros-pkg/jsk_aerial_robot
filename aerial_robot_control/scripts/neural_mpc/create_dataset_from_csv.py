import os, sys
import time
import json
import numpy as np
import pandas as pd
from config.configurations import DirectoryConfig, ModelFitConfig
from utils.data_utils import safe_mkdir_recursive, jsonify, safe_mkfile_recursive
from sim_environment.forward_prop import init_forward_prop, forward_prop
from utils.filter_utils import moving_average_filter, low_pass_filter

# Only needed to get T_samp and params for metadata
sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from neural_controller import NeuralMPC
from config.configurations import EnvConfig
model_options = EnvConfig.model_options
model_options["only_use_nominal"] = True
neural_mpc = NeuralMPC(
    model_options,
    EnvConfig.solver_options,
    EnvConfig.sim_options,
    EnvConfig.run_options,
)
T_samp = neural_mpc.params["T_samp"]
T_step = neural_mpc.params["T_step"]

def get_synched_data_from_rosbag(file_path: str, apply_temporal_filter: bool) -> dict:
    """
    Load and read rosbags from csv file.
    Remove NaN values and synchronize topics based on timestamp. The guiding time series is the thrust command.
    Filter out inconsistent timesteps if apply_temporal_filter is set to True. Steps too close together are removed.
    Steps too far apart are artifically shortened to the ideal time step.
    ASSUMPTION: the temporal recording data is inconsistent due to process lag on the real-machine but the measurements
    are in fact accurately time consistent.
    Returns a dictionary with keys as the topic names and messages as values.
    """

    ############## Read csv file ##############
    df = pd.read_csv(file_path)

    # Position
    data_xyz = df[
        [
            "__time",
            "/beetle1/uav/cog/odom/pose/pose/position/x",
            "/beetle1/uav/cog/odom/pose/pose/position/y",
            "/beetle1/uav/cog/odom/pose/pose/position/z",
        ]
    ]
    data_xyz = data_xyz.dropna()

    # Velocity
    data_vel = df[
        [
            "__time",
            "/beetle1/uav/cog/odom/twist/twist/linear/x",
            "/beetle1/uav/cog/odom/twist/twist/linear/y",
            "/beetle1/uav/cog/odom/twist/twist/linear/z",
        ]
    ]
    data_vel = data_vel.dropna()

    # Quaternion
    data_qwxyz = df[
        [
            "__time",
            "/beetle1/uav/cog/odom/pose/pose/orientation/w",
            "/beetle1/uav/cog/odom/pose/pose/orientation/x",
            "/beetle1/uav/cog/odom/pose/pose/orientation/y",
            "/beetle1/uav/cog/odom/pose/pose/orientation/z",
        ]
    ]
    data_qwxyz = data_qwxyz.dropna()

    # Avoid sign flip of the quaternion
    # This is quite important to maintain continuity of the quaternion signal
    qw = data_qwxyz["/beetle1/uav/cog/odom/pose/pose/orientation/w"].to_numpy()
    qx = data_qwxyz["/beetle1/uav/cog/odom/pose/pose/orientation/x"].to_numpy()
    qy = data_qwxyz["/beetle1/uav/cog/odom/pose/pose/orientation/y"].to_numpy()
    qz = data_qwxyz["/beetle1/uav/cog/odom/pose/pose/orientation/z"].to_numpy()
    qw_prev = 1.0
    qx_prev = 0.0
    qy_prev = 0.0
    qz_prev = 0.0
    for i in range(len(qw)):
        qe_c_w = qw[i] * qw_prev + qx[i] * qx_prev + qy[i] * qy_prev + qz[i] * qz_prev
        if qe_c_w < 0:
            # Negate sign of quaternion at i
            qw[i] = -qw[i]
            qx[i] = -qx[i]
            qy[i] = -qy[i]
            qz[i] = -qz[i]

        qw_prev = qw[i]
        qx_prev = qx[i]
        qy_prev = qy[i]
        qz_prev = qz[i]
    data_qwxyz["/beetle1/uav/cog/odom/pose/pose/orientation/w"] = qw
    data_qwxyz["/beetle1/uav/cog/odom/pose/pose/orientation/x"] = qx
    data_qwxyz["/beetle1/uav/cog/odom/pose/pose/orientation/y"] = qy
    data_qwxyz["/beetle1/uav/cog/odom/pose/pose/orientation/z"] = qz

    # Angular velocity
    data_ang_vel = df[
        [
            "__time",
            "/beetle1/uav/cog/odom/twist/twist/angular/x",
            "/beetle1/uav/cog/odom/twist/twist/angular/y",
            "/beetle1/uav/cog/odom/twist/twist/angular/z",
        ]
    ]
    data_ang_vel = data_ang_vel.dropna()

    # Thrust command
    data_thrust_cmd = df[
        [
            "__time",
            "/beetle1/four_axes/command/base_thrust[0]",
            "/beetle1/four_axes/command/base_thrust[1]",
            "/beetle1/four_axes/command/base_thrust[2]",
            "/beetle1/four_axes/command/base_thrust[3]",
        ]
    ]
    data_thrust_cmd = data_thrust_cmd.dropna()

    # Servo angle command
    data_servo_angle_cmd = df[
        [
            "__time",
            "/beetle1/gimbals_ctrl/gimbal1/position",
            "/beetle1/gimbals_ctrl/gimbal2/position",
            "/beetle1/gimbals_ctrl/gimbal3/position",
            "/beetle1/gimbals_ctrl/gimbal4/position",
        ]
    ]
    data_servo_angle_cmd = data_servo_angle_cmd.dropna()

    # Reference position
    data_xyz_ref = df[
        [
            "__time",
            "/beetle1/nmpc/viz_ref/poses[0]/position/x",
            "/beetle1/nmpc/viz_ref/poses[0]/position/y",
            "/beetle1/nmpc/viz_ref/poses[0]/position/z",
        ]
    ]
    data_xyz_ref = data_xyz_ref.dropna()

    # Reference quaternion
    data_qwxyz_ref = df[
        [
            "__time",
            "/beetle1/nmpc/viz_ref/poses[0]/orientation/w",
            "/beetle1/nmpc/viz_ref/poses[0]/orientation/x",
            "/beetle1/nmpc/viz_ref/poses[0]/orientation/y",
            "/beetle1/nmpc/viz_ref/poses[0]/orientation/z",
        ]
    ]
    data_qwxyz_ref = data_qwxyz_ref.dropna()

    # Avoid sign flip of the quaternion
    qwr = data_qwxyz_ref["/beetle1/nmpc/viz_ref/poses[0]/orientation/w"].to_numpy()
    qxr = data_qwxyz_ref["/beetle1/nmpc/viz_ref/poses[0]/orientation/x"].to_numpy()
    qyr = data_qwxyz_ref["/beetle1/nmpc/viz_ref/poses[0]/orientation/y"].to_numpy()
    qzr = data_qwxyz_ref["/beetle1/nmpc/viz_ref/poses[0]/orientation/z"].to_numpy()
    qwr_prev = 1.0
    qxr_prev = 0.0
    qyr_prev = 0.0
    qzr_prev = 0.0
    for i in range(len(qwr)):
        qe_c_w = qwr[i] * qwr_prev + qxr[i] * qxr_prev + qyr[i] * qyr_prev + qzr[i] * qzr_prev
        if qe_c_w < 0:
            # Negate sign of quaternion at i
            qwr[i] = -qwr[i]
            qxr[i] = -qxr[i]
            qyr[i] = -qyr[i]
            qzr[i] = -qzr[i]

        qwr_prev = qwr[i]
        qxr_prev = qxr[i]
        qyr_prev = qyr[i]
        qzr_prev = qzr[i]
    data_qwxyz_ref["/beetle1/nmpc/viz_ref/poses[0]/orientation/w"] = qwr
    data_qwxyz_ref["/beetle1/nmpc/viz_ref/poses[0]/orientation/x"] = qxr
    data_qwxyz_ref["/beetle1/nmpc/viz_ref/poses[0]/orientation/y"] = qyr
    data_qwxyz_ref["/beetle1/nmpc/viz_ref/poses[0]/orientation/z"] = qzr

    ############## Synchronize ##############
    # Interpolate all data to synchronize w.r.t. the time series of the thrust command using np.interp() [from numpy documentation]:
    # "
    # - One-dimensional linear interpolation for monotonically increasing sample points.
    # - Returns the one-dimensional piecewise linear interpolant to a function with given discrete data points (xp, fp), evaluated at x.
    # "

    # Reference timestamps
    t_ref = np.array(data_thrust_cmd["__time"])

    # Position
    t = np.array(data_xyz["__time"])
    data_xyz_interp = pd.DataFrame()
    data_xyz_interp["__time"] = t_ref
    data_xyz_interp["/beetle1/uav/cog/odom/pose/pose/position/x"] = np.interp(
        t_ref, t, data_xyz["/beetle1/uav/cog/odom/pose/pose/position/x"]
    )
    data_xyz_interp["/beetle1/uav/cog/odom/pose/pose/position/y"] = np.interp(
        t_ref, t, data_xyz["/beetle1/uav/cog/odom/pose/pose/position/y"]
    )
    data_xyz_interp["/beetle1/uav/cog/odom/pose/pose/position/z"] = np.interp(
        t_ref, t, data_xyz["/beetle1/uav/cog/odom/pose/pose/position/z"]
    )

    # Velocity
    t = np.array(data_vel["__time"])
    data_vel_interp = pd.DataFrame()
    data_vel_interp["__time"] = t_ref
    data_vel_interp["/beetle1/uav/cog/odom/twist/twist/linear/x"] = np.interp(
        t_ref, t, data_vel["/beetle1/uav/cog/odom/twist/twist/linear/x"]
    )
    data_vel_interp["/beetle1/uav/cog/odom/twist/twist/linear/y"] = np.interp(
        t_ref, t, data_vel["/beetle1/uav/cog/odom/twist/twist/linear/y"]
    )
    data_vel_interp["/beetle1/uav/cog/odom/twist/twist/linear/z"] = np.interp(
        t_ref, t, data_vel["/beetle1/uav/cog/odom/twist/twist/linear/z"]
    )

    # Quaternion
    t = np.array(data_qwxyz["__time"])
    data_qwxyz_interp = pd.DataFrame()
    data_qwxyz_interp["__time"] = t_ref
    data_qwxyz_interp["/beetle1/uav/cog/odom/pose/pose/orientation/w"] = np.interp(
        t_ref, t, data_qwxyz["/beetle1/uav/cog/odom/pose/pose/orientation/w"]
    )
    data_qwxyz_interp["/beetle1/uav/cog/odom/pose/pose/orientation/x"] = np.interp(
        t_ref, t, data_qwxyz["/beetle1/uav/cog/odom/pose/pose/orientation/x"]
    )
    data_qwxyz_interp["/beetle1/uav/cog/odom/pose/pose/orientation/y"] = np.interp(
        t_ref, t, data_qwxyz["/beetle1/uav/cog/odom/pose/pose/orientation/y"]
    )
    data_qwxyz_interp["/beetle1/uav/cog/odom/pose/pose/orientation/z"] = np.interp(
        t_ref, t, data_qwxyz["/beetle1/uav/cog/odom/pose/pose/orientation/z"]
    )

    # Angular velocity
    t = np.array(data_ang_vel["__time"])
    data_ang_vel_interp = pd.DataFrame()
    data_ang_vel_interp["__time"] = t_ref
    data_ang_vel_interp["/beetle1/uav/cog/odom/twist/twist/angular/x"] = np.interp(
        t_ref, t, data_ang_vel["/beetle1/uav/cog/odom/twist/twist/angular/x"]
    )
    data_ang_vel_interp["/beetle1/uav/cog/odom/twist/twist/angular/y"] = np.interp(
        t_ref, t, data_ang_vel["/beetle1/uav/cog/odom/twist/twist/angular/y"]
    )
    data_ang_vel_interp["/beetle1/uav/cog/odom/twist/twist/angular/z"] = np.interp(
        t_ref, t, data_ang_vel["/beetle1/uav/cog/odom/twist/twist/angular/z"]
    )

    # Thrust command
    # NOTE: No need to interpolate since this is the reference time series

    # Servo angle command
    t = np.array(data_servo_angle_cmd["__time"])
    data_servo_angle_cmd_interp = pd.DataFrame()
    data_servo_angle_cmd_interp["__time"] = t_ref
    data_servo_angle_cmd_interp["/beetle1/gimbals_ctrl/gimbal1/position"] = np.interp(
        t_ref, t, data_servo_angle_cmd["/beetle1/gimbals_ctrl/gimbal1/position"]
    )
    data_servo_angle_cmd_interp["/beetle1/gimbals_ctrl/gimbal2/position"] = np.interp(
        t_ref, t, data_servo_angle_cmd["/beetle1/gimbals_ctrl/gimbal2/position"]
    )
    data_servo_angle_cmd_interp["/beetle1/gimbals_ctrl/gimbal3/position"] = np.interp(
        t_ref, t, data_servo_angle_cmd["/beetle1/gimbals_ctrl/gimbal3/position"]
    )
    data_servo_angle_cmd_interp["/beetle1/gimbals_ctrl/gimbal4/position"] = np.interp(
        t_ref, t, data_servo_angle_cmd["/beetle1/gimbals_ctrl/gimbal4/position"]
    )

    # Reference position
    t = np.array(data_xyz_ref["__time"])
    data_xyz_ref_interp = pd.DataFrame()
    data_xyz_ref_interp["__time"] = t_ref
    data_xyz_ref_interp["/beetle1/nmpc/viz_ref/poses[0]/position/x"] = np.interp(
        t_ref, t, data_xyz_ref["/beetle1/nmpc/viz_ref/poses[0]/position/x"]
    )
    data_xyz_ref_interp["/beetle1/nmpc/viz_ref/poses[0]/position/y"] = np.interp(
        t_ref, t, data_xyz_ref["/beetle1/nmpc/viz_ref/poses[0]/position/y"]
    )
    data_xyz_ref_interp["/beetle1/nmpc/viz_ref/poses[0]/position/z"] = np.interp(
        t_ref, t, data_xyz_ref["/beetle1/nmpc/viz_ref/poses[0]/position/z"]
    )

    # Reference quaternion
    t = np.array(data_qwxyz_ref["__time"])
    data_qwxyz_ref_interp = pd.DataFrame()
    data_qwxyz_ref_interp["__time"] = t_ref
    data_qwxyz_ref_interp["/beetle1/nmpc/viz_ref/poses[0]/orientation/w"] = np.interp(
        t_ref, t, data_qwxyz_ref["/beetle1/nmpc/viz_ref/poses[0]/orientation/w"]
    )
    data_qwxyz_ref_interp["/beetle1/nmpc/viz_ref/poses[0]/orientation/x"] = np.interp(
        t_ref, t, data_qwxyz_ref["/beetle1/nmpc/viz_ref/poses[0]/orientation/x"]
    )
    data_qwxyz_ref_interp["/beetle1/nmpc/viz_ref/poses[0]/orientation/y"] = np.interp(
        t_ref, t, data_qwxyz_ref["/beetle1/nmpc/viz_ref/poses[0]/orientation/y"]
    )
    data_qwxyz_ref_interp["/beetle1/nmpc/viz_ref/poses[0]/orientation/z"] = np.interp(
        t_ref, t, data_qwxyz_ref["/beetle1/nmpc/viz_ref/poses[0]/orientation/z"]
    )

    ############## Export data to dictionary ##############
    data = dict()
    data["timestamp"] = t_ref[:, np.newaxis]
    data["dt"] = np.diff(data["timestamp"], axis=0).squeeze()
    if np.any(data["dt"] == 0.0):
        raise ValueError("There are duplicate timestamps in the thrust command data.")

    data["duration"] = float(data["timestamp"][-1] - data["timestamp"][0])

    data["position"] = data_xyz_interp[
        [
            "/beetle1/uav/cog/odom/pose/pose/position/x",
            "/beetle1/uav/cog/odom/pose/pose/position/y",
            "/beetle1/uav/cog/odom/pose/pose/position/z",
        ]
    ].to_numpy()
    data["velocity"] = data_vel_interp[
        [
            "/beetle1/uav/cog/odom/twist/twist/linear/x",
            "/beetle1/uav/cog/odom/twist/twist/linear/y",
            "/beetle1/uav/cog/odom/twist/twist/linear/z",
        ]
    ].to_numpy()
    data["quaternion"] = data_qwxyz_interp[
        [
            "/beetle1/uav/cog/odom/pose/pose/orientation/w",
            "/beetle1/uav/cog/odom/pose/pose/orientation/x",
            "/beetle1/uav/cog/odom/pose/pose/orientation/y",
            "/beetle1/uav/cog/odom/pose/pose/orientation/z",
        ]
    ].to_numpy()
    data["angular_velocity"] = data_ang_vel_interp[
        [
            "/beetle1/uav/cog/odom/twist/twist/angular/x",
            "/beetle1/uav/cog/odom/twist/twist/angular/y",
            "/beetle1/uav/cog/odom/twist/twist/angular/z",
        ]
    ].to_numpy()

    data["thrust_cmd"] = data_thrust_cmd[
        [
            "/beetle1/four_axes/command/base_thrust[0]",
            "/beetle1/four_axes/command/base_thrust[1]",
            "/beetle1/four_axes/command/base_thrust[2]",
            "/beetle1/four_axes/command/base_thrust[3]",
        ]
    ].to_numpy()
    data["servo_angle_cmd"] = data_servo_angle_cmd_interp[
        [
            "/beetle1/gimbals_ctrl/gimbal1/position",
            "/beetle1/gimbals_ctrl/gimbal2/position",
            "/beetle1/gimbals_ctrl/gimbal3/position",
            "/beetle1/gimbals_ctrl/gimbal4/position",
        ]
    ].to_numpy()

    data["position_ref"] = data_xyz_ref_interp[
        [
            "/beetle1/nmpc/viz_ref/poses[0]/position/x",
            "/beetle1/nmpc/viz_ref/poses[0]/position/y",
            "/beetle1/nmpc/viz_ref/poses[0]/position/z",
        ]
    ].to_numpy()
    data["quaternion_ref"] = data_qwxyz_ref_interp[
        [
            "/beetle1/nmpc/viz_ref/poses[0]/orientation/w",
            "/beetle1/nmpc/viz_ref/poses[0]/orientation/x",
            "/beetle1/nmpc/viz_ref/poses[0]/orientation/y",
            "/beetle1/nmpc/viz_ref/poses[0]/orientation/z",
        ]
    ].to_numpy()

    ############## Temporal filtering ##############
    # Filter for inconsistent timesteps in dt
    # This is very important for training neural networks since we learn to predict the residual dynamics from temporal 
    # dependencies in the measurements and outliers with large or small time steps influence the training significantly.
    # Enforce 0.009s < dt < 0.014s, for T_samp = 0.01s
    # NOTE: dt is the distance between timestamp[i] and timestamp[i+1] stored at index i
    if apply_temporal_filter:
        mean_dt = np.mean(data["dt"])
        std_dt = np.std(data["dt"])
        print(
            f"[INFO] Applying temporal filter to data with mean dt = {mean_dt:.2f}s and std dt = {std_dt:.2f}s. ",
            f"Removing all dt under {mean_dt - std_dt * 3:.2f}s and over {mean_dt + std_dt * 3:.2f}s.",
        )

        # 1. Filter out too large timesteps by shortening them to mean_dt
        # NOTE: This only helps if a time step is delayed and the next time step comes right after it.
        # For now only do this once, if the time gaps are persistent they would be propogated until the end.

        # TODO check if makes sense! (also only for too large dt and not both)
        # invalid_idx = np.append(np.where(data["dt"] > mean_dt + std_dt*3)[0], np.where(data["dt"] < mean_dt - std_dt*3)[0])
        # data["timestamp"][invalid_idx + 1] = (data["timestamp"][invalid_idx + 2] + data["timestamp"][invalid_idx]) / 2

        # # Recompute dt
        # data["dt"] = np.diff(data["timestamp"], axis=0).squeeze()
        # print(f"[INFO] Corrected {len(invalid_idx)} too long timesteps from data.")

        # Debug plot:
            # state_idx = 4
            # plt.figure()
            # plt.plot(timestamps_comp, state_in[:,state_idx], marker="x")
            # plt.plot(timestamps[..., np.where(dt>cutoff_dt)], state_in[np.where(dt>cutoff_dt),state_idx], marker="x", color="red")
    

        # 2. Filter out too small timesteps by removing entries
        while True:
            valid_indices = np.where((data["dt"] >= (mean_dt - std_dt*4)))[0]

            if len(data['dt']) - len(valid_indices) == 0:
                break

            for key in data.keys():
                if key not in ["dt", "duration"]:
                    data[key] = data[key][valid_indices + 1, :]
                    data[key] = data[key][:-1, :]  # Truncate last entry to match size of dt since we recompute it later

            # Recompute dt
            print(f"[INFO] Filtered out {len(data['dt']) - len(valid_indices)} invalid timesteps from data.")
            data["dt"] = np.diff(data["timestamp"], axis=0).squeeze()

    # Truncate last entry to match size of dt (since dt is by definition one entry shorter) NOTE: We do this after filtering to avoid indexing out of bounds
    for key in data.keys():
        if key not in ["dt", "duration"]:
            data[key] = data[key][:-1, :]

    return data


def combine_dicts(data_dicts: dict, T_samp: float):
    """
    Combine multiple csv files into one by appending the data.
    Assumes that all files have the same columns.
    """
    combined_data = dict()

    # Number of recordings
    combined_data["num_recordings"] = len(data_dicts)

    # List durations
    combined_data["duration"] = [data["duration"] for data in data_dicts.values()]

    # Start time of each recording
    combined_data["recording_start_idx"] = []

    # Append timestamps
    first = True
    for data in data_dicts.values():
        if first:
            # Mark liftoff
            combined_data["recording_start_idx"].append(0)
            combined_data["timestamp"] = data["timestamp"]
            combined_data["dt"] = data["dt"]
            last_time = data["timestamp"][-1]
            first = False
        else:
            combined_data["recording_start_idx"].append(combined_data["timestamp"].shape[0])
            combined_data["timestamp"] = np.append(
                combined_data["timestamp"],
                # Connect new timestamps to last timestamp + T_samp
                last_time + T_samp + (data["timestamp"] - data["timestamp"][0]),
            )
            last_time = combined_data["timestamp"][-1]
            combined_data["dt"][-1] = T_samp  # Overwrite last dt
            combined_data["dt"] = np.concatenate((combined_data["dt"], data["dt"]))

    # Append other fields
    for key in data_dicts[0].keys():
        if key in ["timestamp", "duration", "dt"]:
            continue
        combined_data[key] = np.concatenate([data[key] for data in data_dicts.values()], axis=0)

    return combined_data


if __name__ == "__main__":
    """
    Create dataset from rosbag
    """
    ############## Configuration ##############
    # Name of the dataset to be created
    ds_name = "NMPCTiltQdServo" + "_" + "real_machine" + "_dataset_TRAIN_WITH_REF_ALL_PROP"  # + "_01"
    ds_dir = os.path.join(DirectoryConfig.DATA_DIR, ds_name)

    apply_temporal_filter = True

    # Select which recordings to process
    rosbag_dir = "~/ros/rosbag_files/csv/csv_vim_4"
    csv_files = [
        # "2024-09-30-15-58-10_hover.csv",
        # "2024-10-01-15-19-19_new_motor_coeff_hover.csv",
        # "2024-10-02-21-56-59_new_new_motor_coeff_hover_setp_0.3_0.6.csv",
        # "2024-10-09-21-14-53_hand_fly_traj_pitch_roll.csv",
        # "2024-10-11-16-57-32_hand_fly_success.csv",
        # "2024-10-11-17-34-15_success_omni_pitch_rolling.csv",
        # "2024-10-12-10-45-20_success_omni_roll_rolling.csv",
        # "2024-10-15-18-32-17_-pitch_rolling_success.csv",
        # "2024-11-01-21-30-17_momentum_est_only.csv",
        # "2024-11-01-21-40-17_momentum_est_ctrl.csv",
        # "2024-11-07-16-24-27_ITerm_w_hand_dist.csv",
        # "2024-11-07-16-29-49_mom_w_hand_dist.csv",
        # "2024-11-07-17-08-20_mom_acc_est_only.csv",
        # "2024-11-07-17-16-15_mom_acc_est_ctrl_w_hand_dist.csv",
        # "2024-11-07-17-33-18_acceleration_est_only.csv",
        # "2024-11-12-20-13-50_acc_est_only_w_hand_dist.csv",
        # "2024-11-12-20-24-47_acc_ctrl_w_hand_dist.csv",
        # "2024-11-15-16-02-26_acc_ctrl_pitch=1.0_w_hand_dist.csv",
        # "2024-11-15-17-09-35_acc_ctrl_pitch=1.0_w_hand_dist_correct_branch.csv",
        # "2025-03-21-21-40-15_Roll90degYawRotate_mode_0.csv",
        # "2025-05-02-16-02-46_roll_pitch_mode_0.csv",
        # "2025-09-03-14-31-58_mode_0.csv",
        # "2025-09-03-14-38-30_jojo_ws_hovering_success_mode_0.csv",
        # "2025-09-07-17-54-58_jinjie_ws_hovering_success_mode_0.csv",
        # "2025-09-08-23-06-18_nominal_hovering_mode_0_success.csv",
        # "2025-09-08-23-12-12_hovering_mode_3_success.csv",
        # "2025-09-08-23-20-40_hovering_mode_10_success.csv",
        # "2025-09-10-16-44-59_long_flight_ground_effect_targets_mode_10_solver_error_for_aggressive_target_success.csv",
        ### TRAINING ###
        "2025-09-10-17-09-30_long_flight_ground_effect_targets_mode_10_success_TRAIN_WITH_REF.csv",
        "2025-09-10-18-52-13_multiple_smach_trajs_focus_on_rotation_mode_10_TRAIN_WITH_REF_FULL.csv",
        ### VALIDATION ###
        # "2025-09-10-16-44-59_long_flight_ground_effect_targets_mode_10_solver_error_for_aggressive_target_success_VAL_WITH_REF.csv",
        ### HOVERING & GROUND EFFECT TRAIN ###
        # NOT THIS SINCE TOO AGGRESSIVE: "2025-09-10-16-44-59_long_flight_ground_effect_targets_mode_10_solver_error_for_aggressive_target_success_GROUND_EFFECT_ONLY.csv",
        # "2025-09-10-18-52-13_multiple_smach_trajs_focus_on_rotation_mode_10_GROUND_EFFECT_ONLY.csv",
        # "2025-09-10-17-09-30_long_flight_ground_effect_targets_mode_10_success_GROUND_EFFECT_ONLY.csv",  # TRUNCATED FOR LESS AGGRESSIVE
        ### FULL DATASET ###
        # "2025-09-10-17-09-30_long_flight_ground_effect_targets_mode_10_success_FULL.csv",
        # "2025-09-10-18-52-13_multiple_smach_trajs_focus_on_rotation_mode_10_FULL.csv",
        # "2025-09-08-23-12-12_hovering_mode_3_success_FULL.csv",
        # "2025-09-08-23-20-40_hovering_mode_10_success_FULL.csv",
        ### Debug Recording
        # "2026-01-23-11-37-54_mode_11_circle_crash_instability_from_quick_movement.csv",
    ]
    csv_files = [os.path.join(rosbag_dir, file) for file in csv_files]

    data_dicts = dict()
    print(f"Started loading {len(csv_files)} csvs:")
    for i, csv_file in enumerate(csv_files):
        print(f"Loading {csv_file}...")
        data_dicts[i] = get_synched_data_from_rosbag(csv_file, apply_temporal_filter)

    print(f"Finished loading all csvs!")
    if len(csv_files) > 1:
        # TODO not meaningful for temporal neural networks!!
        print(f"Combining dicts..."),
        data = combine_dicts(data_dicts, T_samp)
        print(f"{len(csv_files)} dictionaries successfully combined!")
    else:
        data = data_dicts[0]
        data["recording_start_idx"] = [0]

    ############## Prepare data ##############
    # Time step
    timestamp = data["timestamp"]
    dt = data["dt"]
    # Adjust for size of state_out
    timestamp = timestamp[:-1]
    dt = dt[:-1]

    # State in (adjust for size of state_out)
    # TODO here should be the servo angle state but since its not recorded we use the command
    state_in = np.hstack(
        (
            data["position"][:-1, :],
            data["velocity"][:-1, :],
            data["quaternion"][:-1, :],
            data["angular_velocity"][:-1, :],
            data["servo_angle_cmd"][:-1, :],
        )
    )

    # State out
    # NOTE: State out is the next time step in the time series, meaning the state of the system after T_samp (= dt) seconds
    # By this time the last input command has been applied for dt seconds and the state has changed according to the dynamics
    state_out = np.hstack(
        (
            data["position"][1:, :],
            data["velocity"][1:, :],
            data["quaternion"][1:, :],
            data["angular_velocity"][1:, :],
            data["servo_angle_cmd"][1:, :],
        )
    )

    # Control input (adjust for size of state_out)
    control = np.hstack(
        (
            data["thrust_cmd"][:-1, :],
            data["servo_angle_cmd"][:-1, :],
        )
    )

    # Reference (adjust for size of state_out)
    # TODO Get full state reference from rosbag and then fix visualize_everything.py
    pos_ref = data["position_ref"][:-1, :]
    quat_ref = data["quaternion_ref"][:-1, :]

    # ================= FORWARD PROPAGATION =================
    # Simulate the inside of the MPC which only assumes the nominal model without disturbances
    # state_prop is the propagated state after dt seconds, meaning the state of the system
    # after T_samp (= dt) seconds. By this time the last input command has been applied for dt seconds
    # and the state has changed according to the dynamics.
    # The step size T_prop_step can technically be chosen arbitrary but it makes a lot of sense to choose it as the frequency of the controller (i.e., T_step)
    # since we want to replicate how the model performs inside the MPC prediction scheme. In acados the solver is set up with total horion time
    # solver_options.tf and the number of steps solver_options.N, leading to a step size of T_step = tf / N. But we define the step size and
    # the prediction horizon in the robots ROS package under config and compute the corresponding number of steps. 
    # With T_step = 0.1s and T_horizon = 2.0s, we have N = 20 steps.
    # The propagation time step (meaning how fine we discretize the continuous dynamics) can be chosen independently but it also makes 
    # sense to choose T_samp ≈ dt (= 0.01s)

    #### Short without filtering ####
    print("Forward propagation step size: dt[t]")
    print("Average time step dt in dataset: ", np.mean(dt))
    print("Running forward prop...")

    state_prop_short = np.zeros((0, state_in.shape[1]))
    for t in range(state_in.shape[0]):
        T_prop_horizon_short = dt[t]
        T_prop_step_short = T_prop_horizon_short
        discretized_dynamics_short = init_forward_prop(neural_mpc, T_prop_step=T_prop_step_short, num_stages=4)

        state_curr = state_in[t, :]
        u_cmd = control[t, :]

        state_prop_curr = forward_prop(
            discretized_dynamics_short,
            state_curr[np.newaxis, :],
            u_cmd[np.newaxis, :],
            T_prop_horizon=T_prop_horizon_short,
            T_prop_step=T_prop_step_short,
        )
        state_prop_curr = state_prop_curr[-1, :]
        state_prop_short = np.append(state_prop_short, state_prop_curr[np.newaxis, :], axis=0)

    # Shift state_prop by one timestep to match timestamps of state_out
    state_prop_short = state_prop_short[1:, :]
    print("Finished forward propagation!")

    #### Short with avg filtered input ####
    print("Running forward prop...")
    state_in_avg_filtered = moving_average_filter(state_in, window_size=ModelFitConfig.window_size)
    control_avg_filtered = moving_average_filter(control, window_size=ModelFitConfig.window_size)
    
    state_prop_short_avg_in = np.zeros((0, state_in.shape[1]))
    for t in range(state_in.shape[0]):
        T_prop_horizon_short = dt[t]
        T_prop_step_short = T_prop_horizon_short
        discretized_dynamics_short = init_forward_prop(neural_mpc, T_prop_step=T_prop_step_short, num_stages=4)

        state_curr = state_in_avg_filtered[t, :]
        u_cmd = control_avg_filtered[t, :]

        state_prop_curr = forward_prop(
            discretized_dynamics_short,
            state_curr[np.newaxis, :],
            u_cmd[np.newaxis, :],
            T_prop_horizon=T_prop_horizon_short,
            T_prop_step=T_prop_step_short,
        )
        state_prop_curr = state_prop_curr[-1, :]
        state_prop_short_avg_in = np.append(state_prop_short_avg_in, state_prop_curr[np.newaxis, :], axis=0)

    # Shift state_prop by one timestep to match timestamps of state_out
    state_prop_short_avg_in = state_prop_short_avg_in[1:, :]
    print("Finished forward propagation!")

    #### Short with low pass filtered input ####
    print("Running forward prop...")
    fs = 1.0 / np.mean(dt)
    cutoff = ModelFitConfig.low_pass_filter_cutoff_input
    state_in_low_pass_filtered = np.zeros_like(state_in)
    control_low_pass_filtered = np.zeros_like(control)
    for dim in range(state_in.shape[1]):
        state_in_low_pass_filtered[:, dim] = low_pass_filter(state_in[:, dim], cutoff=cutoff, fs=fs)
    for dim in range(control.shape[1]):
        control_low_pass_filtered[:, dim] = low_pass_filter(control[:, dim], cutoff=cutoff, fs=fs)
    
    state_prop_short_low_pass_in = np.zeros((0, state_in.shape[1]))
    for t in range(state_in.shape[0]):
        T_prop_horizon_short = dt[t]
        T_prop_step_short = T_prop_horizon_short
        discretized_dynamics_short = init_forward_prop(neural_mpc, T_prop_step=T_prop_step_short, num_stages=4)

        state_curr = state_in_low_pass_filtered[t, :]
        u_cmd = control_low_pass_filtered[t, :]

        state_prop_curr = forward_prop(
            discretized_dynamics_short,
            state_curr[np.newaxis, :],
            u_cmd[np.newaxis, :],
            T_prop_horizon=T_prop_horizon_short,
            T_prop_step=T_prop_step_short,
        )
        state_prop_curr = state_prop_curr[-1, :]
        state_prop_short_low_pass_in = np.append(state_prop_short_low_pass_in, state_prop_curr[np.newaxis, :], axis=0)
    
    # Shift state_prop by one timestep to match timestamps of state_out
    state_prop_short_low_pass_in = state_prop_short_low_pass_in[1:, :]
    print("Finished forward propagation!")

    #### Long without filtering ####    
    T_prop_horizon_long = 0.1  #T_step  # 0.1s
    T_prop_step_long = T_prop_horizon_long
    discretized_dynamics_long = init_forward_prop(neural_mpc, T_prop_step=T_prop_step_long, num_stages=4)
    print("Forward propagation step size: ", T_prop_step_long)
    print("Average time step dt in dataset: ", np.mean(dt))
    print("Running forward prop...")

    state_prop_long = np.zeros((0, state_in.shape[1]))
    for t in range(state_in.shape[0]):
        state_curr = state_in[t, :]
        u_cmd = control[t, :]

        state_prop_curr = forward_prop(
            discretized_dynamics_long,
            state_curr[np.newaxis, :],
            u_cmd[np.newaxis, :],
            T_prop_horizon=T_prop_horizon_long,
            T_prop_step=T_prop_step_long,
            const_input=True,
        )
        state_prop_curr = state_prop_curr[-1, :]
        state_prop_long = np.append(state_prop_long, state_prop_curr[np.newaxis, :], axis=0)

    # Shift state_prop by one timestep to match timestamps of state_out
    state_prop_long = state_prop_long[10:, :]  # T_step / dt = 10
    print("Finished forward propagation!")

    #### Long with avg filtered input ####
    print("Running forward prop...")

    state_prop_long_avg_in = np.zeros((0, state_in.shape[1]))
    for t in range(state_in.shape[0]):
        state_curr = state_in_avg_filtered[t, :]
        u_cmd = control_avg_filtered[t, :]

        state_prop_curr = forward_prop(
            discretized_dynamics_long,
            state_curr[np.newaxis, :],
            u_cmd[np.newaxis, :],
            T_prop_horizon=T_prop_horizon_long,
            T_prop_step=T_prop_step_long
        )
        state_prop_curr = state_prop_curr[-1, :]
        state_prop_long_avg_in = np.append(state_prop_long_avg_in, state_prop_curr[np.newaxis, :], axis=0)

    # Shift state_prop by one timestep to match timestamps of state_out
    state_prop_long_avg_in = state_prop_long_avg_in[10:, :]
    print("Finished forward propagation!")

    #### Long with low pass filtered input ####
    print("Running forward prop...")

    state_prop_long_low_pass_in = np.zeros((0, state_in.shape[1]))
    for t in range(state_in.shape[0]):
        state_curr = state_in_low_pass_filtered[t, :]
        u_cmd = control_low_pass_filtered[t, :]

        # Propagate forward
        state_prop_curr = forward_prop(
            discretized_dynamics_long,
            state_curr[np.newaxis, :],
            u_cmd[np.newaxis, :],
            T_prop_horizon=T_prop_horizon_long,
            T_prop_step=T_prop_step_long
        )
        state_prop_curr = state_prop_curr[-1, :]
        state_prop_long_low_pass_in = np.append(state_prop_long_low_pass_in, state_prop_curr[np.newaxis, :], axis=0)

    # Shift state_prop by one timestep to match timestamps of state_out
    state_prop_long_low_pass_in = state_prop_long_low_pass_in[10:, :]
    print("Finished forward propagation!")

    #### MPC propagation without filtering ####
    print(f"[MPC] Using step size: {T_step}")
    model_options = EnvConfig.model_options
    model_options["only_use_nominal"] = True
    neural_mpc = NeuralMPC(
        model_options,
        EnvConfig.solver_options,
        EnvConfig.sim_options,
        EnvConfig.run_options,
    )
    ocp_solver = neural_mpc.get_ocp_solver()
    reference_generator = neural_mpc.get_reference_generator()
    # Warm up solver
    for _ in range(5):
        __ = ocp_solver.solve_for_x0(state_in[0, :])

    u_cmd_solve = np.zeros_like(control)
    state_solve = np.zeros_like(state_in)
    state_ref_trajs = np.zeros_like(state_in)
    control_ref_trajs = np.zeros_like(control)
    for t in range(state_in.shape[0]):
        state_ref, control_ref = reference_generator.compute_trajectory(
            target_xyz=pos_ref[t, :], target_rpy=quat_ref[t, :]
        )
        state_ref_trajs[t, :] = state_ref[0, :]
        control_ref_trajs[t, :] = control_ref[0, :]
        neural_mpc.track(ocp_solver, state_ref, control_ref, None)

        state_curr = state_in[t, :]

        u_cmd_solve[t, :] = ocp_solver.solve_for_x0(state_curr)
        state_solve[t, :] = ocp_solver.get(1, "x")

    u_cmd_solve = u_cmd_solve[int(T_step/0.01):, :]
    state_solve = state_solve[int(T_step/0.01):, :]

    #### MPC propagation with avg filtered input ####
    model_options = EnvConfig.model_options
    model_options["only_use_nominal"] = True
    neural_mpc = NeuralMPC(
        model_options,
        EnvConfig.solver_options,
        EnvConfig.sim_options,
        EnvConfig.run_options,
    )
    ocp_solver = neural_mpc.get_ocp_solver()
    reference_generator = neural_mpc.get_reference_generator()
    # Warm up solver
    for _ in range(5):
        __ = ocp_solver.solve_for_x0(state_in[0, :])

    u_cmd_solve_avg_in = np.zeros_like(control)
    state_solve_avg_in = np.zeros_like(state_in)
    for t in range(state_in.shape[0]):
        state_ref, control_ref = reference_generator.compute_trajectory(
            target_xyz=pos_ref[t, :], target_rpy=quat_ref[t, :]
        )
        neural_mpc.track(ocp_solver, state_ref, control_ref, None)

        state_curr = state_in_avg_filtered[t, :]

        u_cmd_solve_avg_in[t, :] = ocp_solver.solve_for_x0(state_curr)
        state_solve_avg_in[t, :] = ocp_solver.get(1, "x")

    u_cmd_solve_avg_in = u_cmd_solve_avg_in[int(T_step/0.01):, :]
    state_solve_avg_in = state_solve_avg_in[int(T_step/0.01):, :]

    #### MPC propagation with low pass filtered input ####
    model_options = EnvConfig.model_options
    model_options["only_use_nominal"] = True
    neural_mpc = NeuralMPC(
        model_options,
        EnvConfig.solver_options,
        EnvConfig.sim_options,
        EnvConfig.run_options,
    )
    ocp_solver = neural_mpc.get_ocp_solver()
    reference_generator = neural_mpc.get_reference_generator()
    # Warm up solver
    for _ in range(5):
        __ = ocp_solver.solve_for_x0(state_in[0, :])

    u_cmd_solve_low_pass_in = np.zeros_like(control)
    state_solve_low_pass_in = np.zeros_like(state_in)
    for t in range(state_in.shape[0]):
        state_ref, control_ref = reference_generator.compute_trajectory(
            target_xyz=pos_ref[t, :], target_rpy=quat_ref[t, :]
        )
        neural_mpc.track(ocp_solver, state_ref, control_ref, None)

        state_curr = state_in_low_pass_filtered[t, :]

        u_cmd_solve_low_pass_in[t, :] = ocp_solver.solve_for_x0(state_curr)
        state_solve_low_pass_in[t, :] = ocp_solver.get(1, "x")

    u_cmd_solve_low_pass_in = u_cmd_solve_low_pass_in[int(T_step/0.01):, :]
    state_solve_low_pass_in = state_solve_low_pass_in[int(T_step/0.01):, :]

    # Truncate all variables to same length
    timestamp = timestamp[:-10]
    dt = dt[:-10]
    state_in = state_in[:-10, :]
    state_out = state_out[:-10, :]
    control = control[:-10, :]
    pos_ref = pos_ref[:-10, :]
    quat_ref = quat_ref[:-10, :]


    state_prop_short = state_prop_short[:-9, :]
    state_prop_short_avg_in = state_prop_short_avg_in[:-9, :]
    state_prop_short_low_pass_in = state_prop_short_low_pass_in[:-9, :]

    state_ref_trajs = state_ref_trajs[:-10, :]
    control_ref_trajs = control_ref_trajs[:-10, :]
    state_solve = state_solve[:len(state_solve)-10+int(T_step/0.01), :]
    state_solve_avg_in = state_solve_avg_in[:len(state_solve)-10+int(T_step/0.01), :]
    state_solve_low_pass_in = state_solve_low_pass_in[:len(state_solve)-10+int(T_step/0.01), :]
    u_cmd_solve = u_cmd_solve[:len(state_solve)-10+int(T_step/0.01), :]
    u_cmd_solve_avg_in = u_cmd_solve_avg_in[:len(state_solve)-10+int(T_step/0.01), :]
    u_cmd_solve_low_pass_in = u_cmd_solve_low_pass_in[:len(state_solve)-10+int(T_step/0.01), :]

    # 2.1 Check that when using the input for propagation the control input is used from the correct time step
    # -> should be the control input at the same time from which the state_in is measured!
    
    # WHY IS MPC OUTPUT DIFFERENT FROM PROPAGATION!!!

    # USE Varying INPUT FOR PROPAGATION?!

    # 3. warum hat label am anfang so einen offset in ax ay
    print("Forward propagation finished!")
    # ========================================================

    ############## Metadata ##############
    # TODO move file naming and creation into data_utils and generalize
    # TODO make smarter!
    outer_fields = {
        "date": time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime()),
        "real_machine": True,
        "rosbag_file": csv_files,
        "duration": data["duration"],
        "temporal_filtering": apply_temporal_filter,
        "mpc_type": "NMPCTiltQdServo",
        "T_step": T_step,
        "T_samp": T_samp,
        "state_dim": neural_mpc.get_ocp().dims.nx,
        "control_dim": neural_mpc.get_ocp().dims.nu,
        "include_quaternion_constraint": neural_mpc.include_quaternion_constraint,
        "include_soft_constraints": neural_mpc.include_soft_constraints,
        "mpc_params": neural_mpc.params,
        "filtering": {
            "window_size": ModelFitConfig.window_size,
            "low_pass_filter_cutoff_input": ModelFitConfig.low_pass_filter_cutoff_input,
        },
    }
    inner_fields = {
        "disturbances": {
            "cog_dist": False,  # Disturbance forces and torques on CoG
            "cog_dist_model": "",
            "cog_dist_factor": 0.0,
            "motor_noise": False,  # Asymmetric noise in the rotor thrust and servo angles
            "drag": False,  # 2nd order polynomial aerodynamic drag effect
            "payload": False,  # Payload force in the Z axis
        },
    }

    if os.path.exists(ds_dir):
        ds_instances = []
        for _, _, file_names in os.walk(ds_dir):
            ds_instances.extend([os.path.splitext(file)[0] for file in file_names if not file.startswith(".")])

        # Increment counter for dataset file name
        if ds_instances:
            existing_instances = [int(instance.split("_")[1]) for instance in ds_instances]
            max_instance_number = max(existing_instances)
            ds_instance = "dataset_" + str(max_instance_number + 1).zfill(3)
        else:
            ds_instance = "dataset_001"
    else:
        safe_mkdir_recursive(ds_dir)
        ds_instance = "dataset_001"

    is_blank = safe_mkfile_recursive(ds_dir, ds_instance + ".csv")
    if not is_blank:
        raise FileExistsError(
            "Recording file already exists. Please change the dataset instance name or set overwrite to True."
        )

    # Update metadata json file
    json_file_name = os.path.join(DirectoryConfig.DATA_DIR, "metadata.json")
    if os.path.exists(json_file_name):
        with open(json_file_name, "r") as json_file:
            metadata = json.load(json_file)
        if ds_name not in metadata:
            metadata[ds_name] = outer_fields
        metadata[ds_name][ds_instance] = inner_fields

        # Write updated metadata to file
        with open(json_file_name, "w") as json_file:
            json.dump(metadata, json_file, indent=4)
    else:
        # Metadata file does not exist yet
        with open(json_file_name, "w") as json_file:
            ds_instance_name = "dataset_001"
            metadata = {ds_name: {**outer_fields, ds_instance_name: inner_fields}}
            json.dump(metadata, json_file, indent=4)

    ############## Save dataset ##############
    dataset_dict = {
        "timestamp": timestamp,
        "dt": dt,
        # "comp_time": np.zeros((0, 1)),
        # "target": np.zeros((0, target_dim)),
        "state_in": state_in,
        "state_out": state_out,
        "state_prop_short": state_prop_short,
        "state_prop_short_avg_in": state_prop_short_avg_in,
        "state_prop_short_low_pass_in": state_prop_short_low_pass_in,
        "state_prop_long": state_prop_long,
        "state_prop_long_avg_in": state_prop_long_avg_in,
        "state_prop_long_low_pass_in": state_prop_long_low_pass_in,
        "state_solve": state_solve,
        "state_solve_avg_in": state_solve_avg_in,
        "state_solve_low_pass_in": state_solve_low_pass_in,
        "control": control,
        "u_cmd_solve": u_cmd_solve,
        "u_cmd_solve_avg_in": u_cmd_solve_avg_in,
        "u_cmd_solve_low_pass_in": u_cmd_solve_low_pass_in,
        "position_ref": pos_ref,
        "quaternion_ref": quat_ref,
        "state_ref_trajs": state_ref_trajs,
        "control_ref_trajs": control_ref_trajs,
    }
    if len(dataset_dict["state_in"]) > len(dataset_dict["state_out"]):
        raise ValueError("Recording dictionary is not consistent.")

    # Generate new CSV to store data in
    rec_json = dict()
    for key in dataset_dict.keys():
        rec_json[key] = jsonify(dataset_dict[key])

    df = pd.DataFrame(rec_json)
    df.to_csv(os.path.join(ds_dir, ds_instance + ".csv"), index=False, header=True)
    print(f"Saved {ds_instance} in {ds_dir}.")

    # Plot dataset
    # x = np.concatenate((state_in, control), axis=1)
    # y = (state_out - state_prop) / dt[:, np.newaxis]
    # plot_dataset(x, y, dt, state_in, state_out, state_prop, control, save_file_path=None, save_file_name=None)





    # plot
        # import matplotlib.pyplot as plt
        # from config.configurations import ModelFitConfig
        # fs = 1.0 / np.mean(dt)
        # from utils.filter_utils import low_pass_filter
        # state_in_filtered = np.empty_like(state_in)
        # for dim in range(state_in.shape[1]):
        #     state_in_filtered[:, dim] = low_pass_filter(state_in[:, dim], cutoff=ModelFitConfig.low_pass_filter_cutoff_input, fs=fs)
        # state_out_filtered = np.empty_like(state_in)
        # for dim in range(state_in.shape[1]):
        #     state_out_filtered[:, dim] = low_pass_filter(state_out[:, dim], cutoff=ModelFitConfig.low_pass_filter_cutoff_input, fs=fs)
        # control_filtered = np.empty_like(control)
        # for dim in range(control.shape[1]):
        #     control_filtered[:, dim] = low_pass_filter(control[:, dim], cutoff=ModelFitConfig.low_pass_filter_cutoff_input, fs=fs)

        # state_prop_filtered = np.zeros((0, state_in.shape[1]))
        # for t in range(num_iter): #state_in.shape[0]):
        #     # Get current state and control
        #     state_curr = state_in_filtered[t, :]
        #     u_cmd = control_filtered[t, :]

        #     # Propogate forward
        #     state_prop_curr = forward_prop(
        #         discretized_dynamics,
        #         state_curr[np.newaxis, :],
        #         u_cmd[np.newaxis, :],
        #         T_prop_horizon=T_prop_horizon,
        #         T_prop_step=T_prop_step,
        #     )
        #     state_prop_curr = state_prop_curr[-1, :]  # Get last predicted state
        #     state_prop_filtered = np.append(state_prop_filtered, state_prop_curr[np.newaxis, :], axis=0)


        # plt.figure()
        # plt.subplot(3,1,1)
        # plt.plot((state_out_filtered[:num_iter,3] - state_prop_filtered[:,3]) / T_samp, label="state_out_filtered - state_prop_filtered")
        # plt.grid()
        # plt.legend()
        # plt.ylabel("Dim: 0")
        # plt.subplot(3,1,2)
        # plt.plot((state_out_filtered[:num_iter,4] - state_prop_filtered[:,4]) / T_samp)
        # plt.grid()
        # plt.ylabel("Dim: 1")
        # plt.subplot(3,1,3)
        # plt.plot((state_out_filtered[:num_iter,5] - state_prop_filtered[:,5]) / T_samp)
        # plt.grid()
        # plt.ylabel("Dim: 2")

        # plt.figure()
        # plt.subplot(3,1,1)
        # plt.plot(state_in[:,3], label="state_in")
        # plt.plot(state_in_filtered[:,3], label="state_in_filtered")
        # plt.plot(state_prop[:,3], label="state_prop")
        # plt.plot(state_prop_filtered[:,3], label="state_prop_filtered")
        # plt.plot(state_out[:,3], label="state_out")
        # plt.plot(state_out_filtered[:,3], label="state_out_filtered")
        # plt.grid()
        # plt.legend()
        # plt.ylabel("Dim: 0")
        # plt.subplot(3,1,2)
        # plt.plot(state_in[:,4])
        # plt.plot(state_in_filtered[:,4])
        # plt.plot(state_prop[:,4])
        # plt.plot(state_prop_filtered[:,4])
        # plt.plot(state_out[:,4])
        # plt.plot(state_out_filtered[:,4])
        # plt.grid()
        # plt.ylabel("Dim: 1")
        # plt.subplot(3,1,3)
        # plt.plot(state_in[:,5])
        # plt.plot(state_in_filtered[:,5])
        # plt.plot(state_prop[:,5])
        # plt.plot(state_prop_filtered[:,5])
        # plt.plot(state_out[:,5])
        # plt.plot(state_out_filtered[:,5])
        # plt.grid()
        # plt.ylabel("Dim: 2")













    #### plot
    # import matplotlib.pyplot as plt
    # plt.figure()
    # plt.subplot(3, 1, 1)
    # plt.plot(timestamp - timestamp[0], state_out[:,3],label="state_out", marker="x")
    # plt.ylabel("vx [m]")
    # plt.grid("on")
    # ax = plt.gca()
    # ax.axes.xaxis.set_ticklabels([])

    # plt.subplot(3, 1, 2)
    # plt.plot(timestamp - timestamp[0], state_out[:,4],label="state_out", marker="x")
    # plt.ylabel("vy [m]")
    # plt.grid("on")
    # ax = plt.gca()
    # ax.axes.xaxis.set_ticklabels([])

    # plt.subplot(3, 1, 3)
    # plt.plot(timestamp - timestamp[0], state_out[:,5],label="state_out", marker="x")
    # plt.ylabel("vz [m]")
    # plt.grid("on")
    # for t in range(state_in.shape[0]):
    #     plt.subplot(3, 1, 1)
    #     plt.plot(timestamp_traj[t,:] - timestamp[0], state_prop_trajs[t,:,3], label="state_prop_trajs", marker="x")
    #     plt.scatter(timestamp[next_idx[t]] - timestamp[0], state_out[next_idx[t],3], color="r")
    #     if t==0:
    #         plt.legend()

    #     plt.subplot(3, 1, 2)
    #     plt.plot(timestamp_traj[t,:] - timestamp[0], state_prop_trajs[t,:,4], label="state_prop_trajs", marker="x")
    #     plt.scatter(timestamp[next_idx[t]] - timestamp[0], state_out[next_idx[t],4], color="r")

    #     if t==0:
    #         plt.legend()

    #     plt.subplot(3, 1, 3)
    #     plt.plot(timestamp_traj[t,:] - timestamp[0], state_prop_trajs[t,:,5], label="state_prop_trajs", marker="x")
    #     plt.scatter(timestamp[next_idx[t]] - timestamp[0], state_out[next_idx[t],5], color="r")

    #     if t==0:
    #         plt.legend()
    #     halt = 1

    


    # Plot
    # import matplotlib.pyplot as plt
    # plt.figure()
    # plt.subplot(4,1,1)
    # plt.title("Comparison of control inputs from MPC solver and dataset")
    # plt.plot(timestamp[:num_iter], u_cmd_solve[:num_iter,0], label="MPC solved thrust")
    # plt.plot(timestamp[:num_iter], control[:num_iter,0], label="Dataset command thrust")
    # plt.legend()
    # plt.ylabel("Thrust 0")
    # plt.grid("on")
    # ax = plt.gca()
    # ax.axes.xaxis.set_ticklabels([])
    # plt.subplot(4,1,2)
    # plt.plot(timestamp[:num_iter], u_cmd_solve[:num_iter,1])
    # plt.plot(timestamp[:num_iter], control[:num_iter,1])
    # plt.ylabel("Thrust 1")
    # plt.grid("on")
    # ax = plt.gca()
    # ax.axes.xaxis.set_ticklabels([])
    # plt.subplot(4,1,3)
    # plt.plot(timestamp[:num_iter], u_cmd_solve[:num_iter,2])
    # plt.plot(timestamp[:num_iter], control[:num_iter,2])
    # plt.ylabel("Thrust 2")
    # plt.grid("on")
    # ax = plt.gca()
    # ax.axes.xaxis.set_ticklabels([])
    # plt.subplot(4,1,4)
    # plt.plot(timestamp[:num_iter], u_cmd_solve[:num_iter,3])
    # plt.plot(timestamp[:num_iter], control[:num_iter,3])
    # plt.ylabel("Thrust 3")
    # plt.grid("on")

    # plt.figure()
    # plt.subplot(4,1,1)
    # plt.title("Comparison of control inputs from MPC solver and dataset")
    # plt.plot(timestamp[:num_iter], u_cmd_solve[:num_iter,4], label="MPC solved servo")
    # plt.plot(timestamp[:num_iter], control[:num_iter,4], label="Dataset command servo")
    # plt.legend()
    # plt.ylabel("Servo 0")
    # plt.grid("on")
    # ax = plt.gca()
    # ax.axes.xaxis.set_ticklabels([])
    # plt.subplot(4,1,2)
    # plt.plot(timestamp[:num_iter], u_cmd_solve[:num_iter,5])
    # plt.plot(timestamp[:num_iter], control[:num_iter,5])
    # plt.ylabel("Servo 1")
    # plt.grid("on")
    # ax = plt.gca()
    # ax.axes.xaxis.set_ticklabels([])
    # plt.subplot(4,1,3)
    # plt.plot(timestamp[:num_iter], u_cmd_solve[:num_iter,6])
    # plt.plot(timestamp[:num_iter], control[:num_iter,6])
    # plt.ylabel("Servo 2")
    # plt.grid("on")
    # ax = plt.gca()
    # ax.axes.xaxis.set_ticklabels([])
    # plt.subplot(4,1,4)
    # plt.plot(timestamp[:num_iter], u_cmd_solve[:num_iter,7])
    # plt.plot(timestamp[:num_iter], control[:num_iter,7])
    # plt.ylabel("Servo 3")
    # plt.grid("on")

    # plt.figure()
    # state_idx = 5
    # plt.plot(timestamp[:num_iter] - timestamp[0], state_solve[:num_iter,state_idx], label="MPC solved x")
    # plt.plot(timestamp[:num_iter] - timestamp[0], state_in[:num_iter,state_idx], label="state_in")
    # plt.plot(timestamp[:num_iter-1] - timestamp[0], state_prop[:num_iter,state_idx], label="state_prop")
    # plt.plot(timestamp[:num_iter] - timestamp[0], state_out[:num_iter,state_idx], label="state_out")
    # plt.plot(timestamp[next_idx[0]:num_iter] - timestamp[0], state_prop_revised[:num_iter-next_idx[0],state_idx], label="state_prop_revised")
    # plt.plot(timestamp[next_idx[0]:num_iter] - timestamp[0], state_out_revised[:num_iter-next_idx[0],state_idx], label="state_out_revised")
    # plt.legend()
    # plt.grid()
    # plt.title("Comparison of state from MPC solver and dataset")

    # plt.figure()
    # plt.plot(timestamp[:num_iter-1], state_out[:num_iter-1,state_idx] - state_prop[:num_iter,state_idx], label="state_out - state_prop")
    # plt.plot(timestamp[:num_iter], state_out_revised[:num_iter,state_idx] - state_prop_revised[:num_iter,state_idx], label="state_out_revised - state_prop_revised")
    # plt.legend()
    # plt.grid()

