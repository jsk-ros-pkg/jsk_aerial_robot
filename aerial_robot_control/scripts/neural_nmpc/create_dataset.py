import os, sys
import numpy as np
import pandas as pd
import json
import time
from pathlib import Path
from config.configurations import DirectoryConfig
from utils.data_utils import safe_mkdir_recursive, jsonify, safe_mkfile_recursive
from sim_environment.forward_prop import init_forward_prop, forward_prop

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from nmpc.nmpc_tilt_mt.tilt_qd import phys_param_beetle_omni as phys_omni
from nmpc.nmpc_tilt_mt.tilt_qd.tilt_qd_servo import NMPCTiltQdServo


class struct(object):
    pass


def numerically_differentiate(data: np.ndarray, dt: float) -> np.ndarray:
    # TODO implement
    dp = (data[:, 0:3] - data[:, 0:3]) / dt
    dq = 1


def read_rosbag(file_path: str):
    """
    Load and read rosbags from file.
    Returns a dictionary with keys as the topic names and messages as values.
    """
    bagpath = Path(file_path)

    # Create a type store to use if the bag has no message definitions.
    typestore = get_typestore(Stores.ROS2_FOXY)

    topic_list = [
        # "/beetle1/nmpc/viz_pred",
        "/beetle1/uav/cog/odom",
        "/beetle1/four_axes/command",
        "/beetle1/gimbals_ctrl",
    ]

    data = {
        "timestamp": np.zeros((0, 1)),
        "duration": 0.0,
        "dt": np.zeros((0, 1)),
        "position": np.zeros((0, 3)),
        "velocity": np.zeros((0, 3)),
        "quaternion": np.zeros((0, 4)),
        "angular_velocity": np.zeros((0, 3)),
        "thrust_cmd": np.zeros((0, 4)),
        "servo_angle_cmd": np.zeros((0, 4)),
    }

    # Create reader instance and open for reading
    with AnyReader([bagpath], default_typestore=typestore) as reader:
        connections = [x for x in reader.connections if x.topic in topic_list]
        data["duration"] = reader.duration / 1000 / 1000 / 1000  # in seconds

        last_timestamp = reader.start_time

        trigger = True
        stored_from_odom = False
        stored_from_thrust = False
        stored_from_servo = False

        quat_prev = np.array([1.0, 0.0, 0.0, 0.0])
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            # if connection.topic == "/beetle1/nmpc/viz_pred":
            # if not all_stored:
            #     trigger = True
            # else:
            #     trigger = False
            #     all_stored = False
            #     stored_from_odom = False
            #     stored_from_thrust = False
            #     stored_from_servo = False

            if trigger:
                data["timestamp"] = np.vstack((data["timestamp"], timestamp))
                trigger = False

            if connection.topic == "/beetle1/uav/cog/odom" and not stored_from_odom:
                stored_from_odom = True
                # --- Position
                # From first node in MPC prediction horizon
                # Convert geometry_msgs/msg/PoseArray to geometry_msgs/msg/Pose to geometry_msgs/msg/Point to x y z
                x = msg.pose.pose.position.x
                y = msg.pose.pose.position.y
                z = msg.pose.pose.position.z
                position = np.array([x, y, z])
                data["position"] = np.vstack((data["position"], position))

                # --- Quaternion
                # From first node in MPC prediction horizon
                # Convert geometry_msgs/msg/PoseArray to geometry_msgs/msg/Pose to geometry_msgs/msg/Quaternion to x y z w
                qw = msg.pose.pose.orientation.w
                qx = msg.pose.pose.orientation.x
                qy = msg.pose.pose.orientation.y
                qz = msg.pose.pose.orientation.z
                quaternion = np.array([qw, qx, qy, qz])

                # === check the sign of the quaternion, avoid the flip of the quaternion ===
                # This is quite important because of the continuity of the quaternion
                qe_c_w = qw * quat_prev[0] + qx * quat_prev[1] + qy * quat_prev[2] + qz * quat_prev[3]
                if qe_c_w < 0:
                    quaternion = -quaternion

                quat_prev = quaternion

                data["quaternion"] = np.vstack((data["quaternion"], quaternion))

                # TODO odom data has noise at the end of the flight -> record better
                # --- Velocity
                # Convert geometry_msgs/msg/TwistWithCovariance to geometry_msgs/msg/Twist to geometry_msgs/msg/Vector3 to x y z
                vx = msg.twist.twist.linear.x
                vy = msg.twist.twist.linear.y
                vz = msg.twist.twist.linear.z
                velocity = np.array([vx, vy, vz])
                data["velocity"] = np.vstack((data["velocity"], velocity))

                # --- Angular velocity
                # Convert geometry_msgs/msg/TwistWithCovariance to geometry_msgs/msg/Twist to geometry_msgs/msg/Vector3 to x y z
                roll_rate = msg.twist.twist.angular.x
                pitch_rate = msg.twist.twist.angular.y
                yaw_rate = msg.twist.twist.angular.z
                angular_velocity = np.array([roll_rate, pitch_rate, yaw_rate])
                data["angular_velocity"] = np.vstack((data["angular_velocity"], angular_velocity))

            elif connection.topic == "/beetle1/four_axes/command" and not stored_from_thrust:
                stored_from_thrust = True
                # --- Thrust command
                # Convert std_msgs/msg/Float32MultiArray to numpy array
                thrust_cmd = msg.base_thrust
                data["thrust_cmd"] = np.vstack((data["thrust_cmd"], thrust_cmd))

            elif connection.topic == "/beetle1/gimbals_ctrl" and not stored_from_servo:
                stored_from_servo = True
                # --- Servo angle command
                # Convert std_msgs/msg/Float32MultiArray to numpy array
                servo_angle_cmd = msg.position
                data["servo_angle_cmd"] = np.vstack((data["servo_angle_cmd"], servo_angle_cmd))

            if stored_from_odom and stored_from_thrust and stored_from_servo:
                # --- Time step
                # NOTE: Use time stamp from thrust command to get real T_samp of NMPC
                # Compute time step from timestamp difference
                dt = (timestamp - last_timestamp) / 1000 / 1000 / 1000  # in seconds
                last_timestamp = timestamp
                data["dt"] = np.vstack((data["dt"], dt))

                trigger = True
                stored_from_odom = False
                stored_from_thrust = False
                stored_from_servo = False

    # Ignore first entry since it is from initialization
    data["timestamp"] = data["timestamp"][1:, :]
    data["dt"] = data["dt"][1:, :]
    data["position"] = data["position"][1:, :]
    data["velocity"] = data["velocity"][1:, :]
    data["quaternion"] = data["quaternion"][1:, :]
    data["angular_velocity"] = data["angular_velocity"][1:, :]
    data["thrust_cmd"] = data["thrust_cmd"][1:, :]
    data["servo_angle_cmd"] = data["servo_angle_cmd"][1:, :]

    return data


if __name__ == "__main__":
    """
    Create dataset from rosbag
    """
    rosbag_file = "/home/johannes/ros/rosbag_files/2025-03-21-21-40-15_Roll90degYawRotate.bag"

    data = read_rosbag(rosbag_file)

    # TODO move file naming and creation into data_utils and generalize
    ds_name = "NMPCTiltQdServo" + "_" + "real_machine" + "_dataset" + "_01"
    ds_dir = os.path.join(DirectoryConfig.DATA_DIR, ds_name)

    # TODO make smarter!
    nmpc = NMPCTiltQdServo(phys=phys_omni)  # JUST FOR PARAMS TO WRITE IN METADATA!
    outer_fields = {
        "date": time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime()),
        "real_machine": True,
        "rosbag_file": rosbag_file,
        "duration": data["duration"],
        "nmpc_type": "NMPCTiltQdServo",
        "state_dim": nmpc.get_ocp().dims.nx,
        "control_dim": nmpc.get_ocp().dims.nu,
        "include_quaternion_constraint": nmpc.include_quaternion_constraint,
        "include_soft_constraints": nmpc.include_soft_constraints,
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
    # NOTE: state_out is the next time step in the time series
    state_out = np.hstack(
        (
            data["position"][1:, :],
            data["velocity"][1:, :],
            data["quaternion"][1:, :],
            data["angular_velocity"][1:, :],
            data["servo_angle_cmd"][1:, :],
        )
    )
    state_prop = np.zeros((0, state_in.shape[1]))
    control = np.hstack((data["thrust_cmd"][:, :], data["servo_angle_cmd"][:-1, :]))
    dt = data["dt"]

    # === Simulate the inside of the NMPC which only assumes the nominal model without disturbances ===
    # --- Initialize forward propagation
    # Replicate important properties for nominal model
    nmpc = struct()
    nmpc.tilt = True
    nmpc.include_servo_model = True
    nmpc.include_thrust_model = False
    nmpc.include_servo_derivative = False
    nmpc.phys = phys_omni

    # Define nominal model
    dynamics_forward_prop, state_forward_prop, u_forward_prop = init_forward_prop(nmpc)

    # --- Simulation parameters
    T_prop = 0.005  # Basically arbitrary but makes sense to choose as T_sim

    # --- Run forward propagation with nominal model based with state_in and control
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

    # Create dictionary that will become dataset
    dataset_dict = {
        # "timestamp": np.zeros((0, 1)),
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
    df.to_csv(os.path.join(ds_dir, ds_instance, ".csv"), index=False, header=True)
