"""
 Created by li-jinjie on 25-6-13.
"""

import pandas as pd
import numpy as np
import scienceplots
import matplotlib.pyplot as plt
import argparse
from utils import quat2euler, calculate_rmse

legend_alpha = 0.5


def main(file_path):
    # Load the data from csv file
    data = pd.read_csv(file_path)

    # ======= xyz =========
    data_xyz = data[
        [
            "__time",
            "/beetle1/uav/cog/odom/pose/pose/position/x",
            "/beetle1/uav/cog/odom/pose/pose/position/y",
            "/beetle1/uav/cog/odom/pose/pose/position/z",
        ]
    ]

    try:
        data_xyz_ref = data[
            [
                "__time",
                "/beetle1/set_ref_traj/x/data[0]",
                "/beetle1/set_ref_traj/x/data[1]",
                "/beetle1/set_ref_traj/x/data[2]",
            ]
        ]
    except KeyError:
        # assign the reference trajectory to zero
        data_xyz_ref = pd.DataFrame()
        data_xyz_ref["__time"] = data_xyz["__time"]
        data_xyz_ref["/beetle1/set_ref_traj/x/data[0]"] = -0.095
        data_xyz_ref["/beetle1/set_ref_traj/x/data[1]"] = -0.015
        data_xyz_ref["/beetle1/set_ref_traj/x/data[2]"] = 0.6

    data_xyz = data_xyz.dropna()
    data_xyz_ref = data_xyz_ref.dropna()

    # ======= rpy =========
    data_qwxyz = data[
        [
            "__time",
            "/beetle1/uav/cog/odom/pose/pose/orientation/w",
            "/beetle1/uav/cog/odom/pose/pose/orientation/x",
            "/beetle1/uav/cog/odom/pose/pose/orientation/y",
            "/beetle1/uav/cog/odom/pose/pose/orientation/z",
        ]
    ]

    try:
        data_qwxyz_ref = data[
            [
                "__time",
                "/beetle1/set_ref_traj/x/data[6]",
                "/beetle1/set_ref_traj/x/data[7]",
                "/beetle1/set_ref_traj/x/data[8]",
                "/beetle1/set_ref_traj/x/data[9]",
            ]
        ]
    except KeyError:
        # assign the reference trajectory to zero
        data_qwxyz_ref = pd.DataFrame()
        data_qwxyz_ref["__time"] = data_qwxyz["__time"]
        data_qwxyz_ref["/beetle1/set_ref_traj/x/data[6]"] = 0
        data_qwxyz_ref["/beetle1/set_ref_traj/x/data[7]"] = 0
        data_qwxyz_ref["/beetle1/set_ref_traj/x/data[8]"] = 0
        data_qwxyz_ref["/beetle1/set_ref_traj/x/data[9]"] = 0

    data_qwxyz_ref = data_qwxyz_ref.dropna()
    data_qwxyz = data_qwxyz.dropna()

    # convert to euler
    data_euler_ref = pd.DataFrame()
    data_euler_ref["__time"] = data_qwxyz_ref["__time"]
    data_euler_ref["roll"], data_euler_ref["pitch"], data_euler_ref["yaw"] = quat2euler(
        data_qwxyz_ref["/beetle1/set_ref_traj/x/data[6]"],
        data_qwxyz_ref["/beetle1/set_ref_traj/x/data[7]"],
        data_qwxyz_ref["/beetle1/set_ref_traj/x/data[8]"],
        data_qwxyz_ref["/beetle1/set_ref_traj/x/data[9]"],
        sequence="ZYX",
        degrees=False,
    )

    data_euler = pd.DataFrame()
    data_euler["__time"] = data_qwxyz["__time"]
    data_euler = pd.DataFrame()
    data_euler["__time"] = data_qwxyz["__time"]
    data_euler["roll"], data_euler["pitch"], data_euler["yaw"] = quat2euler(
        data_qwxyz["/beetle1/uav/cog/odom/pose/pose/orientation/w"],
        data_qwxyz["/beetle1/uav/cog/odom/pose/pose/orientation/x"],
        data_qwxyz["/beetle1/uav/cog/odom/pose/pose/orientation/y"],
        data_qwxyz["/beetle1/uav/cog/odom/pose/pose/orientation/z"],
        sequence="ZYX",
        degrees=False,
    )

    # thrust_cmd
    data_thrust_cmd = data[
        [
            "__time",
            "/beetle1/four_axes/command/base_thrust[0]",
            "/beetle1/four_axes/command/base_thrust[1]",
            "/beetle1/four_axes/command/base_thrust[2]",
            "/beetle1/four_axes/command/base_thrust[3]",
        ]
    ]
    data_thrust_cmd = data_thrust_cmd.dropna()

    # servo angle cmd
    data_servo_angle_cmd = data[
        [
            "__time",
            "/beetle1/gimbals_ctrl/gimbal1/position",
            "/beetle1/gimbals_ctrl/gimbal2/position",
            "/beetle1/gimbals_ctrl/gimbal3/position",
            "/beetle1/gimbals_ctrl/gimbal4/position",
        ]
    ]
    data_servo_angle_cmd = data_servo_angle_cmd.dropna()

    # # real servo angle
    # data_servo_angle = data[
    #     ['__time', '/beetle1/joint_states/gimbal1/position', '/beetle1/joint_states/gimbal2/position',
    #      '/beetle1/joint_states/gimbal3/position', '/beetle1/joint_states/gimbal4/position']]
    # data_servo_angle = data_servo_angle.dropna()

    # ============ Plotting ==============
    plt.style.use(["science", "grid"])

    plt.rcParams.update({"font.size": 11})  # default is 10
    label_size = 14

    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(3.5, 3.5))

    t_bias = max(data_xyz["__time"].iloc[0], data_xyz_ref["__time"].iloc[0])
    color_ref = "#0C5DA5"
    color_real = "#FF2C00"

    # --------------------------------
    plt.subplot(2, 1, 1)
    t_ref = np.array(data_xyz_ref["__time"]) - t_bias
    x_ref = np.array(data_xyz_ref["/beetle1/set_ref_traj/x/data[0]"])
    # plt.plot(t_ref, x_ref, label='X$_r$', linestyle="--")

    t = np.array(data_xyz["__time"]) - t_bias
    x = np.array(data_xyz["/beetle1/uav/cog/odom/pose/pose/position/x"])
    plt.plot(t, x, label="X", linestyle="-")

    # calculate RMSE
    rmse_x = calculate_rmse(t, x, t_ref, x_ref)
    print(f"RMSE X (m): {rmse_x}")

    # ------
    t_ref = np.array(data_xyz_ref["__time"]) - t_bias
    y_ref = np.array(data_xyz_ref["/beetle1/set_ref_traj/x/data[1]"])
    # plt.plot(t_ref, y_ref, label='Y$_r$', linestyle="--")

    t = np.array(data_xyz["__time"]) - t_bias
    y = np.array(data_xyz["/beetle1/uav/cog/odom/pose/pose/position/y"])
    plt.plot(t, y, label="Y", linestyle="--")

    # calculate RMSE
    rmse_y = calculate_rmse(t, y, t_ref, y_ref)
    print(f"RMSE Y (m): {rmse_y}")

    # ------
    t_ref = np.array(data_xyz_ref["__time"]) - t_bias
    z_ref = np.array(data_xyz_ref["/beetle1/set_ref_traj/x/data[2]"])
    # plt.plot(t_ref, z_ref, label='Z$_r$', linestyle="--")

    t = np.array(data_xyz["__time"]) - t_bias
    z = np.array(data_xyz["/beetle1/uav/cog/odom/pose/pose/position/z"])

    plt.plot(t, z, label="Z", linestyle="-.")

    # calculate RMSE
    rmse_z = calculate_rmse(t, z, t_ref, z_ref)
    print(f"RMSE Z (m): {rmse_z}")

    plt.ylabel("Position [m]", fontsize=label_size)
    # plt.xlim(0, 28)
    plt.ylim(0.6, 1.1)
    plt.legend(framealpha=legend_alpha, loc="center", ncol=3)

    # --------------------------------
    plt.subplot(2, 1, 2)
    t_ref = np.array(data_euler_ref["__time"]) - t_bias
    roll_ref = np.array(data_euler_ref["roll"])
    # plt.plot(t_ref, roll_ref * 180 / np.pi, label='R_r', linestyle="--")

    t = np.array(data_euler["__time"]) - t_bias
    roll = np.array(data_euler["roll"])
    plt.plot(t, roll * 180 / np.pi, label="R", linestyle="-")

    # calculate RMSE
    rmse_roll = calculate_rmse(t, roll, t_ref, roll_ref)
    print(f"RMSE Roll (rad): {rmse_roll}")
    print(f"RMSE Roll (deg): {rmse_roll * 180 / np.pi}")

    # ------
    t_ref = np.array(data_euler_ref["__time"]) - t_bias
    pitch_ref = np.array(data_euler_ref["pitch"])
    # plt.plot(t_ref, pitch_ref * 180 / np.pi, label='P_r', linestyle="--")

    t = np.array(data_euler["__time"]) - t_bias
    pitch = np.array(data_euler["pitch"])
    plt.plot(t, pitch * 180 / np.pi, label="P", linestyle="--")

    # calculate RMSE
    rmse_pitch = calculate_rmse(t, pitch, t_ref, pitch_ref)
    print(f"RMSE Pitch (rad): {rmse_pitch}")
    print(f"RMSE Pitch (deg): {rmse_pitch * 180 / np.pi}")

    # ------
    t_ref = np.array(data_euler_ref["__time"]) - t_bias
    yaw_ref = np.array(data_euler_ref["yaw"])
    # if yaw_ref has a jump, we need to fix it
    for i in range(1, len(yaw_ref)):
        if yaw_ref[i] - yaw_ref[i - 1] > np.pi:
            yaw_ref[i:] -= 2 * np.pi
        elif yaw_ref[i] - yaw_ref[i - 1] < -np.pi:
            yaw_ref[i:] += 2 * np.pi
    # plt.plot(t_ref, yaw_ref * 180 / np.pi, label='Y_r', linestyle="--")

    t = np.array(data_euler["__time"]) - t_bias
    yaw = np.array(data_euler["yaw"])
    # if yaw has a jump, we need to fix it
    for i in range(1, len(yaw)):
        if yaw[i] - yaw[i - 1] > np.pi:
            yaw[i:] -= 2 * np.pi
        elif yaw[i] - yaw[i - 1] < -np.pi:
            yaw[i:] += 2 * np.pi
    plt.plot(t, yaw * 180 / np.pi, label="Y", linestyle="-.")

    # calculate RMSE
    rmse_yaw = calculate_rmse(t, yaw, t_ref, yaw_ref, is_yaw=True)
    print(f"RMSE Yaw (rad): {rmse_yaw}")
    print(f"RMSE Yaw (deg): {rmse_yaw * 180 / np.pi}")

    plt.ylabel("Attitude [$^\\circ$]", fontsize=label_size)
    plt.ylim(-11, 6)

    # plt.xlim(0, 28)
    plt.xlabel("Time [s]", fontsize=label_size)

    plt.legend(framealpha=legend_alpha, ncol=3, loc="lower center")

    # --------------------------------
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Plot the trajectory. Please use plotjuggler to generate the csv file."
    )
    parser.add_argument("file_path", type=str, help="The file name of the trajectory")

    args = parser.parse_args()

    main(args.file_path)
