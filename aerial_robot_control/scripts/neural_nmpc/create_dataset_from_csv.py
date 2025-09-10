import os, sys
import numpy as np
import pandas as pd
import json
import time
from pathlib import Path
from config.configurations import DirectoryConfig
from utils.data_utils import safe_mkdir_recursive, jsonify, safe_mkfile_recursive
from utils.visualization_utils import plot_dataset
from sim_environment.forward_prop import init_forward_prop, forward_prop

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from nmpc.nmpc_tilt_mt.tilt_qd import phys_param_beetle_omni as phys_omni
from nmpc.nmpc_tilt_mt.tilt_qd.tilt_qd_servo import NMPCTiltQdServo

# def read_rosbag(file_path: str):
#     """
#     Load and read rosbags from file.
#     Returns a dictionary with keys as the topic names and messages as values.
#     """
#     bagpath = Path(file_path)

#     # Create a type store to use if the bag has no message definitions.
#     typestore = get_typestore(Stores.ROS2_FOXY)

#     topic_list = [
#         "/beetle1/uav/cog/odom",
#         "/beetle1/four_axes/command",
#         "/beetle1/gimbals_ctrl",
#     ]
#     data = {
#         "timestamp": np.zeros((0, 1)),
#         "dt": np.zeros((0, 1)),
#         "position": np.zeros((0, 3)),
#         "velocity": np.zeros((0, 3)),
#         "quaternion": np.zeros((0, 4)),
#         "angular_velocity": np.zeros((0, 3)),
#         "thrust_cmd": np.zeros((0, 4)),
#         "servo_angle_cmd": np.zeros((0, 4)),
#     }

#     # Create reader instance and open for reading
#     with AnyReader([bagpath], default_typestore=typestore) as reader:
#         connections = [x for x in reader.connections if x.topic in topic_list]
#         for connection, timestamp, rawdata in reader.messages(connections=connections):
#             msg = reader.deserialize(rawdata, connection.msgtype)
#             if connection.topic == "/beetle1/uav/cog/odom":
#                 data["position"] = np.vstack((data["position"], position))

#                 # --- Quaternion
#                 # From first node in MPC prediction horizon
#                 # Convert geometry_msgs/msg/PoseArray to geometry_msgs/msg/Pose to geometry_msgs/msg/Quaternion to x y z w
#                 qw = msg.pose.pose.orientation.w
#                 qx = msg.pose.pose.orientation.x
#                 qy = msg.pose.pose.orientation.y
#                 qz = msg.pose.pose.orientation.z
#                 quaternion = np.array([qw, qx, qy, qz])

#                 # === check the sign of the quaternion, avoid the flip of the quaternion ===
#                 # This is quite important because of the continuity of the quaternion
#                 qe_c_w = qw * quat_prev[0] + qx * quat_prev[1] + qy * quat_prev[2] + qz * quat_prev[3]
#                 if qe_c_w < 0:
#                     quaternion = -quaternion

#                 quat_prev = quaternion

#                 data["quaternion"] = np.vstack((data["quaternion"], quaternion))

#                 # TODO odom data has noise at the end of the flight -> record better
#                 # --- Velocity
#                 # Convert geometry_msgs/msg/TwistWithCovariance to geometry_msgs/msg/Twist to geometry_msgs/msg/Vector3 to x y z
#                 vx = msg.twist.twist.linear.x
#                 vy = msg.twist.twist.linear.y
#                 vz = msg.twist.twist.linear.z
#                 velocity = np.array([vx, vy, vz])
#                 data["velocity"] = np.vstack((data["velocity"], velocity))

#                 # --- Angular velocity
#                 # Convert geometry_msgs/msg/TwistWithCovariance to geometry_msgs/msg/Twist to geometry_msgs/msg/Vector3 to x y z
#                 roll_rate = msg.twist.twist.angular.x
#                 pitch_rate = msg.twist.twist.angular.y
#                 yaw_rate = msg.twist.twist.angular.z
#                 angular_velocity = np.array([roll_rate, pitch_rate, yaw_rate])
#                 data["angular_velocity"] = np.vstack((data["angular_velocity"], angular_velocity))

#             elif connection.topic == "/beetle1/four_axes/command" and not stored_from_thrust:
#                 stored_from_thrust = True
#                 # --- Thrust command
#                 # Convert std_msgs/msg/Float32MultiArray to numpy array
#                 thrust_cmd = msg.base_thrust
#                 data["thrust_cmd"] = np.vstack((data["thrust_cmd"], thrust_cmd))

#             elif connection.topic == "/beetle1/gimbals_ctrl" and not stored_from_servo:
#                 stored_from_servo = True
#                 # --- Servo angle command
#                 # Convert std_msgs/msg/Float32MultiArray to numpy array
#                 servo_angle_cmd = msg.position
#                 data["servo_angle_cmd"] = np.vstack((data["servo_angle_cmd"], servo_angle_cmd))

#             if stored_from_odom and stored_from_thrust and stored_from_servo:
#                 # --- Time step
#                 # NOTE: Use time stamp from thrust command to get real T_samp of NMPC
#                 # Compute time step from timestamp difference
#                 dt = (timestamp - last_timestamp) / 1000 / 1000 / 1000  # in seconds
#                 last_timestamp = timestamp
#                 data["dt"] = np.vstack((data["dt"], dt))

#                 trigger = True
#                 stored_from_odom = False
#                 stored_from_thrust = False
#                 stored_from_servo = False

#     # Ignore first entry since it is from initialization
#     data["timestamp"] = data["timestamp"][1:, :]
#     data["dt"] = data["dt"][1:, :]
#     data["position"] = data["position"][1:, :]
#     data["velocity"] = data["velocity"][1:, :]
#     data["quaternion"] = data["quaternion"][1:, :]
#     data["angular_velocity"] = data["angular_velocity"][1:, :]
#     data["thrust_cmd"] = data["thrust_cmd"][1:, :]
#     data["servo_angle_cmd"] = data["servo_angle_cmd"][1:, :]

#     return data


def get_synched_data_from_rosbag(file_path: str):
    """
    Load and read rosbags from csv file.
    Remove NaN values and synchronize topics based on timestamp. The guiding time series is the thrust command.
    Returns a dictionary with keys as the topic names and messages as values.
    """

    ############## Read csv file ##############
    df = pd.read_csv(file_path)

    # --- Position
    data_xyz = df[
        [
            "__time",
            "/beetle1/uav/cog/odom/pose/pose/position/x",
            "/beetle1/uav/cog/odom/pose/pose/position/y",
            "/beetle1/uav/cog/odom/pose/pose/position/z",
        ]
    ]
    data_xyz = data_xyz.dropna()

    # --- Velocity
    data_vel = df[
        [
            "__time",
            "/beetle1/uav/cog/odom/twist/twist/linear/x",
            "/beetle1/uav/cog/odom/twist/twist/linear/y",
            "/beetle1/uav/cog/odom/twist/twist/linear/z",
        ]
    ]
    data_vel = data_vel.dropna()

    # --- Quaternion
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

    # === check the sign of the quaternion, avoid the flip of the quaternion ===
    # This is quite important because of the continuity of the quaternion
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

    # --- Angular velocity
    data_ang_vel = df[
        [
            "__time",
            "/beetle1/uav/cog/odom/twist/twist/angular/x",
            "/beetle1/uav/cog/odom/twist/twist/angular/y",
            "/beetle1/uav/cog/odom/twist/twist/angular/z",
        ]
    ]
    data_ang_vel = data_ang_vel.dropna()

    # --- Thrust command
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

    # --- Servo angle command
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

    ############## SYNCHRONIZE ##############
    # Interpolate all data to the time series of the thrust command
    # Using np.interp() [from numpy documentation]:
    # - "One-dimensional linear interpolation for monotonically increasing sample points.
    # - Returns the one-dimensional piecewise linear interpolant to a function with given discrete data points (xp, fp), evaluated at x."

    # --- Reference timestamps
    t_ref = np.array(data_thrust_cmd["__time"])

    # --- Position
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

    # --- Velocity
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

    # --- Quaternion
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

    # --- Angular velocity
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

    # --- Thrust command
    # No need to interpolate since this is the reference time series

    # --- Servo angle command
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

    ############## Export data to dictionary ##############
    data = dict()
    data["timestamp"] = t_ref
    data["dt"] = np.diff(data["timestamp"])
    if np.any(data["dt"] == 0.0):
        raise ValueError("There are duplicate timestamps in the thrust command data.")

    data["duration"] = data["timestamp"][-1] - data["timestamp"][0]

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

    # Append timestamps
    first = True
    for data in data_dicts.values():
        if first:
            combined_data["timestamp"] = data["timestamp"]
            combined_data["dt"] = data["dt"]
            last_time = data["timestamp"][-1]
            first = False
        else:
            combined_data["timestamp"] = np.append(
                combined_data["timestamp"],
                # Connect new timestamps to last timestamp + T_samp
                last_time + T_samp + (data["timestamp"] - data["timestamp"][0]),
            )
            combined_data["dt"] = np.concatenate((combined_data["dt"], np.array([T_samp]), data["dt"]))

    # Append other fields
    for key in data_dicts[0].keys():
        if key in ["timestamp", "duration", "dt"]:
            continue
        combined_data[key] = np.concatenate([data[key] for data in data_dicts.values()], axis=0)

    # # Correct binding segments in timestamp
    # combined_data["dt"][len(data_dicts[0]["dt"])-1] = T_samp

    # NOTE: Timestamps needs to be truncated to match size of state_out
    combined_data["timestamp"] = combined_data["timestamp"][:-1]
    return combined_data


if __name__ == "__main__":
    """
    Create dataset from rosbag
    """
    nmpc = NMPCTiltQdServo(phys=phys_omni, build=False)  # JUST FOR PARAMS TO WRITE IN METADATA! AND T_SAMP
    T_samp = nmpc.params["T_samp"]

    rosbag_dir = "/home/johannes/ros/rosbag_files/csv"
    csv_files = [
        "2024-09-30-15-58-10_hover.csv",
        "2024-10-01-15-19-19_new_motor_coeff_hover.csv",
        "2024-10-02-21-56-59_new_new_motor_coeff_hover_setp_0.3_0.6.csv",
        "2024-10-09-21-14-53_hand_fly_traj_pitch_roll.csv",
        "2024-10-11-16-57-32_hand_fly_success.csv",
        "2024-10-11-17-34-15_success_omni_pitch_rolling.csv",
        "2024-10-12-10-45-20_success_omni_roll_rolling.csv",
        "2024-10-15-18-32-17_-pitch_rolling_success.csv",
        "2024-11-01-21-30-17_momentum_est_only.csv",
        "2024-11-01-21-40-17_momentum_est_ctrl.csv",
        "2024-11-07-16-24-27_ITerm_w_hand_dist.csv",
        "2024-11-07-16-29-49_mom_w_hand_dist.csv",
        "2024-11-07-17-08-20_mom_acc_est_only.csv",
        "2024-11-07-17-16-15_mom_acc_est_ctrl_w_hand_dist.csv",
        "2024-11-07-17-33-18_acceleration_est_only.csv",
        "2024-11-12-20-13-50_acc_est_only_w_hand_dist.csv",
        "2024-11-12-20-24-47_acc_ctrl_w_hand_dist.csv",
        "2024-11-15-16-02-26_acc_ctrl_pitch=1.0_w_hand_dist.csv",
        "2024-11-15-17-09-35_acc_ctrl_pitch=1.0_w_hand_dist_correct_branch.csv",
        "2025-03-21-21-40-15_Roll90degYawRotate_mode_0.csv",
        "2025-05-02-16-02-46_roll_pitch_mode_0.csv",
        "2025-09-03-14-31-58_mode_0.csv",
        "2025-09-03-14-38-30_jojo_ws_hovering_success_mode_0.csv",
        "2025-09-07-17-54-58_jinjie_ws_hovering_success_mode_0.csv",
        "2025-09-08-23-06-18_nominal_hovering_mode_0_success.csv",
        "2025-09-08-23-12-12_hovering_mode_3_success.csv",
        "2025-09-08-23-20-40_hovering_mode_10_success.csv",
    ]
    csv_files = [os.path.join(rosbag_dir, file) for file in csv_files]

    data_dicts = dict()
    print(f"Started loading {len(csv_files)} csvs.")
    for i, csv_file in enumerate(csv_files):
        print(f"Loading {csv_file}...")
        data_dicts[i] = get_synched_data_from_rosbag(csv_file)

    print(f"Finished loading all csvs!")
    print(f"Combining dicts...")
    if len(csv_files) > 1:
        # TODO not meaningful for temporal neural networks!!
        data = combine_dicts(data_dicts, T_samp)
    else:
        data = data_dicts[0]
    print("Dictionaries successfully combined!")

    # TODO move file naming and creation into data_utils and generalize
    ds_name = "NMPCTiltQdServo" + "_" + "real_machine" + "_dataset" + "_01"
    ds_dir = os.path.join(DirectoryConfig.DATA_DIR, ds_name)

    # TODO make smarter!
    outer_fields = {
        "date": time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime()),
        "real_machine": True,
        "rosbag_file": csv_file,
        "duration": data["duration"],
        "nmpc_type": "NMPCTiltQdServo",
        "state_dim": nmpc.get_ocp().dims.nx,
        "control_dim": nmpc.get_ocp().dims.nu,
        "include_quaternion_constraint": nmpc.include_quaternion_constraint,
        "include_soft_constraints": nmpc.include_soft_constraints,
        "nmpc_params": nmpc.params,
    }
    inner_fields = {
        "disturbances": {
            "cog_dist": False,  # Disturbance forces and torques on CoG
            "cog_dist_model": "mu = 1 / (z+1)**2 * cog_dist_factor * max_thrust * 4 / std = 0",
            "cog_dist_factor": 0.1,
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

    # Assemble state and control arrays
    # --- Time step
    timestamp = data["timestamp"]
    dt = data["dt"]

    # --- State in
    # NOTE: state_in is the current time step in the time series but needs to be cut short by one at the end to match state_out
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

    # --- State out
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

    # --- Control input
    control = np.hstack((data["thrust_cmd"][:-1, :], data["servo_angle_cmd"][:-1, :]))

    # --- State prop
    # Propagate state_in and control input through nominal model to get state_prop
    state_prop = np.zeros((0, state_in.shape[1]))

    # === Simulate the inside of the NMPC which only assumes the nominal model without disturbances ===
    # - Initialize forward propagation
    # Define nominal model
    dynamics_forward_prop, state_forward_prop, u_forward_prop = init_forward_prop(nmpc)

    # - Simulation parameters
    T_prop = 0.005  # Basically arbitrary but makes sense to choose as T_sim (or half of T_samp, i.e., half of dt)
    print("Forward propagation step size: ", T_prop)
    print("Average time step dt in dataset: ", np.mean(dt))
    print("Running forward prop...")
    # - Run forward propagation with nominal model based with state_in and control
    for t in range(state_in.shape[0]):
        # Get current state and control
        state_curr = state_in[t, :]
        u_cmd = control[t, :]

        # Propogate forward
        state_prop_curr = forward_prop(
            dynamics_forward_prop,
            state_forward_prop,
            u_forward_prop,
            state_curr[np.newaxis, :],
            u_cmd[np.newaxis, :],
            T_horizon=dt[t],
            T_step=T_prop,
            num_stages=4,
        )
        state_prop_curr = state_prop_curr[-1, :]  # Get last predicted state
        state_prop = np.append(state_prop, state_prop_curr[np.newaxis, :], axis=0)
    # =================================================================================================
    print("Forward propagation finished!")

    # Create dictionary that will become dataset
    dataset_dict = {
        "timestamp": timestamp,
        "dt": dt,
        # "comp_time": np.zeros((0, 1)),
        # "target": np.zeros((0, target_dim)),
        "state_in": state_in,
        "state_out": state_out,
        "state_prop": state_prop,
        "control": control,
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

    # # Plot dataset
    # # Labels
    # x = np.concatenate((state_in, control), axis=1)
    # y = (state_out - state_prop) / dt[:, np.newaxis]
    # plot_dataset(x, y, dt, state_in, state_out, state_prop, control, save_file_path=None, save_file_name=None)
