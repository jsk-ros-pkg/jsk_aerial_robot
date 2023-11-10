import argparse
from pathlib import Path
from rosbags.highlevel import AnyReader
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation


def plot_all_curves(x_list, y_list, z_list, roll_deg_list, pitch_deg_list, yaw_deg_list):
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


def process_data(inertial_name, data_name, time_list, data_list, alpha_value, static_value=None):
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

    for i in range(len(intersection_time) - 1):
        print(f"{data_name} intersection time interval {i}: {intersection_time[i + 1] - intersection_time[i]} s")
    average_half_interval = np.mean(intersection_time[1:] - intersection_time[:-1])
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
    I = m * g * (D**2) * (T**2) / (16 * (np.pi**2) * L)
    print(f"{inertial_name} calculating by {data_name}: {I} kg*m^2")


L = 1.920  # m
D = 0.176  # m
m = 1.086  # kg
g = 9.798  # m/s^2  Tokyo

flag_plot = False

# Press the green button in the gutter to run the script.
if __name__ == "__main__":
    # get rosbag file path
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--rosbag_file_path",
        help="rosbag file path",
        default="/home/lijinjie/ROS1/jsk_aerial_robot_ws/rosbags/Ixx_data.bag",
    )
    args = parser.parse_args()

    time_list = []
    x_list = []
    y_list = []
    z_list = []
    roll_deg_list = []
    pitch_deg_list = []
    yaw_deg_list = []

    time_start = None

    # create reader instance and open for reading
    with AnyReader([Path(args.rosbag_file_path)]) as reader:
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
    if flag_plot:
        plot_all_curves(x_list, y_list, z_list, roll_deg_list, pitch_deg_list, yaw_deg_list)

    # process_data("Ixx", "pos_x", time_list, x_list, 0.20)
    # process_data("Ixx", "pos_y", time_list, y_list, 0.80)
    process_data("Ixx", "roll_deg", time_list, roll_deg_list, 0.80)
