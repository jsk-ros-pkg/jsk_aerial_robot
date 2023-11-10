from __future__ import print_function, annotations
import argparse
from pathlib import Path
from rosbags.highlevel import AnyReader
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation


class RelatedParams:
    def __init__(self):
        self.flag_plot = False
        self.flag_verbose = True
        self.L = 1.920  # m
        self.D = 0.176  # m
        self.m = 1.086  # kg
        self.g = 9.798  # m/s^2  Tokyo


def _plot_all_curves(x_list, y_list, z_list, roll_deg_list, pitch_deg_list, yaw_deg_list):
    plt.figure()

    plt.subplot(2, 3, 1)
    plt.plot(x_list)
    plt.title("x")
    plt.xlabel("time/s")
    plt.ylabel("m")

    plt.subplot(2, 3, 2)
    plt.plot(y_list)
    plt.title("y")
    plt.xlabel("time/s")
    plt.ylabel("m")

    plt.subplot(2, 3, 3)
    plt.plot(z_list)
    plt.title("z")
    plt.xlabel("time/s")
    plt.ylabel("m")

    plt.subplot(2, 3, 4)
    plt.plot(roll_deg_list)
    plt.title("roll")
    plt.xlabel("time/s")
    plt.ylabel("deg")

    plt.subplot(2, 3, 5)
    plt.plot(pitch_deg_list)
    plt.title("pitch")
    plt.xlabel("time/s")
    plt.ylabel("deg")

    plt.subplot(2, 3, 6)
    plt.plot(yaw_deg_list)
    plt.title("yaw")
    plt.xlabel("time/s")
    plt.ylabel("deg")

    # adjust the layout
    plt.tight_layout()

    plt.show()


def _process_data(
    params: RelatedParams,
    rosbag_name: str,
    data_name: str,
    time_list: list,
    data_list: list,
    alpha_value: float,
    static_value: float = None,
):
    # low pass filter
    x_org = np.array(data_list)
    x_filtered = np.zeros_like(x_org)

    alpha = alpha_value
    x_filtered[0] = x_org[0]
    for i in range(1, len(x_org)):
        x_filtered[i] = alpha * x_filtered[i - 1] + (1 - alpha) * x_org[i]

    # plot the filtered data and original data
    x_filtered_mean = np.mean(x_filtered)
    plt.figure()
    plt.title(data_name)
    plt.plot(x_org, label="original")
    plt.plot(x_filtered, label="filtered")
    plt.xlabel("time/s")
    plt.ylabel("m")
    plt.legend()

    plt.axhline(y=x_filtered_mean, color="r", linestyle="-")  # plot the average value

    plt.show()

    # find how many times the x curve intersects the average value
    x = x_filtered
    if static_value is not None:
        x_static = static_value
    else:
        x_static = np.mean(x)
    x_diff = x - x_static
    x_diff_sign = np.sign(x_diff)

    x_diff_sign_change = np.diff(x_diff_sign)
    x_diff_sign_change_nonzero = np.nonzero(x_diff_sign_change)

    intersection_index = np.array(x_diff_sign_change_nonzero[0])

    # calculate the period
    time_np = np.array(time_list)
    intersection_time = (time_np[intersection_index] + time_np[intersection_index + 1]) / 2

    if params.flag_verbose:
        for i in range(len(intersection_time) - 1):
            print(f"{data_name} intersection time interval {i}: {intersection_time[i + 1] - intersection_time[i]} s")
    average_half_interval = np.mean(intersection_time[1:] - intersection_time[:-1])
    if params.flag_verbose:
        print(f"{data_name} average half interval: {average_half_interval} s")

    # check all the intervals is greater than the average_half_interval/2
    for i in range(len(intersection_time) - 1):
        if intersection_time[i + 1] - intersection_time[i] < average_half_interval / 2:
            raise ValueError(
                f"{data_name} intersection time interval {i}: {intersection_time[i + 1] - intersection_time[i]} s "
                f"is less than average_half_interval/2: {average_half_interval / 2} s\n"
                f"Please check the data or consider increasing the influence of filter!"
            )

    # calculate the inertia
    T = 2 * average_half_interval
    I = params.m * params.g * (params.D**2) * (T**2) / (16 * (np.pi**2) * params.L)
    print(f"Calculate inertial parameters through {rosbag_name} with {data_name}:\n" f"I: {I} kg*m^2")


def calculate_inertial(bag_file_path: str, params: RelatedParams, data_name: str, alpha_value: float):
    time_list = []
    x_list = []
    y_list = []
    z_list = []
    roll_deg_list = []
    pitch_deg_list = []
    yaw_deg_list = []

    time_start = None

    # create reader instance and open for reading
    with AnyReader([Path(bag_file_path)]) as reader:
        connections = [x for x in reader.connections if x.topic == "/mocap/pose"]
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)

            if time_start is None:
                time_start = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

            time_list.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 - time_start)

            x_list.append(msg.pose.position.x)
            y_list.append(msg.pose.position.y)
            z_list.append(msg.pose.position.z)

            # convert quaternion to euler angle with scipy.spatial.transform.Rotation
            q = Rotation.from_quat(
                [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
            )
            rot_euler = q.as_euler("xyz", degrees=True)
            roll_deg_list.append(rot_euler[0])
            pitch_deg_list.append(rot_euler[1])
            yaw_deg_list.append(rot_euler[2])

    # plot all data in one figure
    if params.flag_plot:
        _plot_all_curves(x_list, y_list, z_list, roll_deg_list, pitch_deg_list, yaw_deg_list)

    # process_data(args.rosbag_file_path, "pos_x", time_list, x_list, 0.8)
    if data_name == "pos_x":
        _process_data(params, bag_file_path, data_name, time_list, x_list, alpha_value)
    elif data_name == "pos_y":
        _process_data(params, bag_file_path, data_name, time_list, y_list, alpha_value)


if __name__ == "__main__":
    """Identify procedure
    1. Attach the robot to a bifilar pendulum
    2. Rotate it along the vertical axis
    3. Use MoCap to estimate state, and use rosbag to save the COG data
    4. Measure the parameters of the bifilar pendulum
    5. Tell the path-to-rosbag to this script and calculate the inertial parameter
    """

    related_params = RelatedParams()
    related_params.flag_plot = False
    related_params.flag_verbose = False

    # ================================  mini_qd_w_battery_lidar_Ixx.bag  ================================
    file_path_1 = "/home/lijinjie/ROS1/jsk_aerial_robot_ws/rosbags/mini_qd_w_battery_lidar_Ixx.bag"
    calculate_inertial(file_path_1, related_params, data_name="pos_x", alpha_value=0.90)
    calculate_inertial(file_path_1, related_params, data_name="pos_y", alpha_value=0.40)

    file_path_2 = "/home/lijinjie/ROS1/jsk_aerial_robot_ws/rosbags/mini_qd_w_battery_lidar_Ixx_2.bag"
    calculate_inertial(file_path_2, related_params, data_name="pos_x", alpha_value=0.95)
    calculate_inertial(file_path_2, related_params, data_name="pos_y", alpha_value=0.40)

    # ================================  mini_qd_w_battery_lidar_Iyy.bag  ================================
    file_path_3 = "/home/lijinjie/ROS1/jsk_aerial_robot_ws/rosbags/mini_qd_w_battery_lidar_Iyy.bag"
    calculate_inertial(file_path_3, related_params, data_name="pos_x", alpha_value=0.80)

    file_path_4 = "/home/lijinjie/ROS1/jsk_aerial_robot_ws/rosbags/mini_qd_w_battery_lidar_Iyy_2.bag"
    # calculate_inertial(file_path_4, related_params, data_name="pos_x", alpha_value=0.93)  # bad data
    calculate_inertial(file_path_4, related_params, data_name="pos_y", alpha_value=0.95)
