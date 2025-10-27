import pandas as pd
import numpy as np
import scienceplots
import matplotlib.pyplot as plt
import argparse

from matplotlib.lines import lineStyles

from utils import unwrap_angle_sequence, calculate_rmse, quat2euler, calculate_quat_error, interp_quat
from utils import matlab_yellow, matlab_green, matlab_orange, matlab_blue

legend_alpha = 0.5


def main(file_path, type, if_hand_teleop):
    # Load the data from csv file
    data = pd.read_csv(file_path)

    # ======= xyz =========
    data_xyz = data[
        [
            "__time",
            "/beetle1/uav/ee_contact/odom/pose/pose/position/x",
            "/beetle1/uav/ee_contact/odom/pose/pose/position/y",
            "/beetle1/uav/ee_contact/odom/pose/pose/position/z",
        ]
    ]

    data_xyz_cog = data[
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
                "/beetle1/set_ref_traj/points[0]/transforms[0]/translation/x",
                "/beetle1/set_ref_traj/points[0]/transforms[0]/translation/y",
                "/beetle1/set_ref_traj/points[0]/transforms[0]/translation/z",
            ]
        ]
    except KeyError:
        # assign the reference trajectory to zero
        data_xyz_ref = pd.DataFrame()
        data_xyz_ref["__time"] = data_xyz["__time"]
        data_xyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/translation/x"] = 0.0
        data_xyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/translation/y"] = 0.0
        data_xyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/translation/z"] = 1.0

    data_xyz = data_xyz.dropna()
    data_xyz_ref = data_xyz_ref.dropna()
    data_xyz_cog = data_xyz_cog.dropna()

    # ======= rpy =========
    data_qwxyz = data[
        [
            "__time",
            "/beetle1/uav/ee_contact/odom/pose/pose/orientation/w",
            "/beetle1/uav/ee_contact/odom/pose/pose/orientation/x",
            "/beetle1/uav/ee_contact/odom/pose/pose/orientation/y",
            "/beetle1/uav/ee_contact/odom/pose/pose/orientation/z",
        ]
    ]

    data_qwxyz_cog = data[
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
                "/beetle1/set_ref_traj/points[0]/transforms[0]/rotation/w",
                "/beetle1/set_ref_traj/points[0]/transforms[0]/rotation/x",
                "/beetle1/set_ref_traj/points[0]/transforms[0]/rotation/y",
                "/beetle1/set_ref_traj/points[0]/transforms[0]/rotation/z",
            ]
        ]
    except KeyError:
        # assign the reference trajectory to zero
        data_qwxyz_ref = pd.DataFrame()
        data_qwxyz_ref["__time"] = data_qwxyz["__time"]
        data_qwxyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/rotation/w"] = 1.0
        data_qwxyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/rotation/x"] = 0.0
        data_qwxyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/rotation/y"] = 0.0
        data_qwxyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/rotation/z"] = 0.0

    data_qwxyz_ref = data_qwxyz_ref.dropna()
    data_qwxyz = data_qwxyz.dropna()
    data_qwxyz_cog = data_qwxyz_cog.dropna()

    # convert to euler
    data_euler_ref = pd.DataFrame()
    data_euler_ref["__time"] = data_qwxyz_ref["__time"]
    data_euler_ref["roll"], data_euler_ref["pitch"], data_euler_ref["yaw"] = quat2euler(
        data_qwxyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/rotation/w"],
        data_qwxyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/rotation/x"],
        data_qwxyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/rotation/y"],
        data_qwxyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/rotation/z"],
        sequence="ZYX",
        degrees=False,
    )
    data_euler_ref["roll"] = unwrap_angle_sequence(data_euler_ref["roll"].to_numpy())
    data_euler_ref["pitch"] = unwrap_angle_sequence(data_euler_ref["pitch"].to_numpy())
    data_euler_ref["yaw"] = unwrap_angle_sequence(data_euler_ref["yaw"].to_numpy())

    # interpolate the real quaternion date
    t_ref = np.array(data_qwxyz_ref["__time"])
    t = np.array(data_qwxyz["__time"])

    data_qwxyz_interp = interp_quat(t_ref, t, data_qwxyz, "/beetle1/uav/ee_contact/odom/pose/pose/orientation")

    # calculate the quaternion error
    ew, ex, ey, ez = calculate_quat_error(
        data_qwxyz_interp["/beetle1/uav/ee_contact/odom/pose/pose/orientation/w"],
        data_qwxyz_interp["/beetle1/uav/ee_contact/odom/pose/pose/orientation/x"],
        data_qwxyz_interp["/beetle1/uav/ee_contact/odom/pose/pose/orientation/y"],
        data_qwxyz_interp["/beetle1/uav/ee_contact/odom/pose/pose/orientation/z"],
        data_qwxyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/rotation/w"],
        data_qwxyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/rotation/x"],
        data_qwxyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/rotation/y"],
        data_qwxyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/rotation/z"],
    )

    e_roll, e_pitch, e_yaw = quat2euler(ew, ex, ey, ez, sequence="ZYX", degrees=False)

    data_euler = pd.DataFrame()
    data_euler["__time"] = t_ref
    data_euler["roll"] = e_roll.to_numpy() + data_euler_ref["roll"].to_numpy()
    data_euler["pitch"] = e_pitch.to_numpy() + data_euler_ref["pitch"].to_numpy()
    data_euler["yaw"] = e_yaw.to_numpy() + data_euler_ref["yaw"].to_numpy()

    # cog euler
    t_cog = np.array(data_qwxyz_cog["__time"])
    data_qwxyz_cog_interp = interp_quat(t_ref, t_cog, data_qwxyz_cog, "/beetle1/uav/cog/odom/pose/pose/orientation")

    ew_cog, ex_cog, ey_cog, ez_cog = calculate_quat_error(
        data_qwxyz_cog_interp["/beetle1/uav/cog/odom/pose/pose/orientation/w"],
        data_qwxyz_cog_interp["/beetle1/uav/cog/odom/pose/pose/orientation/x"],
        data_qwxyz_cog_interp["/beetle1/uav/cog/odom/pose/pose/orientation/y"],
        data_qwxyz_cog_interp["/beetle1/uav/cog/odom/pose/pose/orientation/z"],
        data_qwxyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/rotation/w"],
        data_qwxyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/rotation/x"],
        data_qwxyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/rotation/y"],
        data_qwxyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/rotation/z"],
    )
    e_roll_cog, e_pitch_cog, e_yaw_cog = quat2euler(ew_cog, ex_cog, ey_cog, ez_cog, sequence="ZYX", degrees=False)

    data_euler_cog = pd.DataFrame()
    data_euler_cog["__time"] = t_ref
    data_euler_cog["roll"] = e_roll_cog.to_numpy() + data_euler_ref["roll"].to_numpy()
    data_euler_cog["pitch"] = e_pitch_cog.to_numpy() + data_euler_ref["pitch"].to_numpy()
    data_euler_cog["yaw"] = e_yaw_cog.to_numpy() + data_euler_ref["yaw"].to_numpy()

    # ======= actuators =========
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

    # ======= est. wrench =========
    try:
        data_iterm = data[
            [
                "__time",
                "/beetle1/dist_w_f_cog_tq/iterm/wrench/force/x",
                "/beetle1/dist_w_f_cog_tq/iterm/wrench/force/y",
                "/beetle1/dist_w_f_cog_tq/iterm/wrench/force/z",
                "/beetle1/dist_w_f_cog_tq/iterm/wrench/torque/x",
                "/beetle1/dist_w_f_cog_tq/iterm/wrench/torque/y",
                "/beetle1/dist_w_f_cog_tq/iterm/wrench/torque/z",
            ]
        ]
        data_iterm = data_iterm.dropna()

        data_ext_pure = data[
            [
                "__time",
                "/beetle1/dist_w_f_cog_tq/ext/wrench/force/x",
                "/beetle1/dist_w_f_cog_tq/ext/wrench/force/y",
                "/beetle1/dist_w_f_cog_tq/ext/wrench/force/z",
                "/beetle1/dist_w_f_cog_tq/ext/wrench/torque/x",
                "/beetle1/dist_w_f_cog_tq/ext/wrench/torque/y",
                "/beetle1/dist_w_f_cog_tq/ext/wrench/torque/z",
            ]
        ]
        data_ext_pure = data_ext_pure.dropna()

        data_ext_wrench_est = data[
            [
                "__time",
                "/beetle1/ext_wrench_est/value/wrench/force/x",
                "/beetle1/ext_wrench_est/value/wrench/force/y",
                "/beetle1/ext_wrench_est/value/wrench/force/z",
                "/beetle1/ext_wrench_est/value/wrench/torque/x",
                "/beetle1/ext_wrench_est/value/wrench/torque/y",
                "/beetle1/ext_wrench_est/value/wrench/torque/z",
            ]
        ]
        data_ext_wrench_est = data_ext_wrench_est.dropna()
    except KeyError:
        print("No est. wrench data found!")

    # ======= plotting =========
    if type == 0:
        plt.style.use(["science", "grid"])

        plt.rcParams.update({"font.size": 11})  # default is 10
        label_size = 14

        fig = plt.figure(figsize=(7, 7))

        t_bias = max(data_xyz["__time"].iloc[0], data_xyz_ref["__time"].iloc[0], data_xyz_cog["__time"].iloc[0])
        color_ref = "#0C5DA5"
        color_real = "#FF2C00"
        color_cog = "#f29619"  # the orange in scienceplots

        inside_valve_t_start = 13.7
        inside_valve_t_stop = 58.9

        con_rot_t_start = 39.0
        con_rot_t_stop = 68.6

        # --------------------------------
        plt.subplot(4, 2, 1)
        t_ref = np.array(data_xyz_ref["__time"]) - t_bias
        x_ref = np.array(data_xyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/translation/x"])
        plt.plot(t_ref, x_ref, label="ref_ee", linestyle="--", color=color_ref)

        t_cog = np.array(data_xyz_cog["__time"]) - t_bias
        x_cog = np.array(data_xyz_cog["/beetle1/uav/cog/odom/pose/pose/position/x"])
        plt.plot(t_cog, x_cog, label="cog", linestyle="-.", color=color_cog)

        t = np.array(data_xyz["__time"]) - t_bias
        x = np.array(data_xyz["/beetle1/uav/ee_contact/odom/pose/pose/position/x"])
        plt.plot(t, x, label="ee", color=color_real)

        if if_hand_teleop:
            plt.axvspan(inside_valve_t_start, inside_valve_t_stop, facecolor=matlab_yellow, alpha=0.2)

        plt.legend(framealpha=legend_alpha, ncol=2)
        plt.ylabel("X [m]", fontsize=label_size)

        # # right Y-axis: error plot
        # ax = plt.gca()
        # ax2 = ax.twinx()
        # error_x = abs(x - np.interp(t, t_ref, x_ref))
        # ax2.plot(t, error_x, label='error', alpha=0.5)
        # ax2.set_ylabel('Err [m]', fontsize=label_size)
        # # ax2.tick_params(axis='y', labelcolor='tab:red')  # change the color of y axis

        # calculate RMSE
        rmse_x = calculate_rmse(t, x, t_ref, x_ref)
        print(f"RMSE X [m]: {rmse_x}")

        # --------------------------------
        plt.subplot(4, 2, 2)
        t_ref = np.array(data_euler_ref["__time"]) - t_bias
        roll_ref = np.array(data_euler_ref["roll"])
        plt.plot(t_ref, roll_ref * 180 / np.pi, label="ref", linestyle="--", color=color_ref)

        t_cog = np.array(data_euler_cog["__time"]) - t_bias
        roll_cog = np.array(data_euler_cog["roll"])
        plt.plot(t_cog, roll_cog * 180 / np.pi, label="cog", linestyle="-.", color=color_cog)

        t = np.array(data_euler["__time"]) - t_bias
        roll = np.array(data_euler["roll"])
        plt.plot(t, roll * 180 / np.pi, label="real", color=color_real)

        plt.ylabel("Roll [$^\\circ$]", fontsize=label_size)

        if if_hand_teleop:
            plt.axvspan(con_rot_t_start, con_rot_t_stop, facecolor=matlab_green, alpha=0.2)
            plt.axvspan(
                con_rot_t_start, con_rot_t_stop, facecolor="none", edgecolor="lightgray", hatch="///", linewidth=0.0
            )

        # calculate RMSE
        rmse_roll = calculate_rmse(t, roll, t_ref, roll_ref)
        print(f"RMSE Roll [rad]: {rmse_roll}")
        print(f"RMSE Roll [deg]: {rmse_roll * 180 / np.pi}")

        # --------------------------------
        plt.subplot(4, 2, 3)
        t_ref = np.array(data_xyz_ref["__time"]) - t_bias
        y_ref = np.array(data_xyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/translation/y"])
        plt.plot(t_ref, y_ref, label="ref_ee", linestyle="--", color=color_ref)

        t_cog = np.array(data_xyz_cog["__time"]) - t_bias
        y_cog = np.array(data_xyz_cog["/beetle1/uav/cog/odom/pose/pose/position/y"])
        plt.plot(t_cog, y_cog, label="cog", linestyle="-.", color=color_cog)

        t = np.array(data_xyz["__time"]) - t_bias
        y = np.array(data_xyz["/beetle1/uav/ee_contact/odom/pose/pose/position/y"])
        plt.plot(t, y, label="ee", color=color_real)
        plt.ylabel("Y [m]", fontsize=label_size)

        if if_hand_teleop:
            plt.axvspan(inside_valve_t_start, inside_valve_t_stop, facecolor=matlab_yellow, alpha=0.2)

        # plt.legend(framealpha=legend_alpha, ncol=2)

        # calculate RMSE
        rmse_y = calculate_rmse(t, y, t_ref, y_ref)
        print(f"RMSE Y [m]: {rmse_y}")

        # --------------------------------
        plt.subplot(4, 2, 4)
        t_ref = np.array(data_euler_ref["__time"]) - t_bias
        pitch_ref = np.array(data_euler_ref["pitch"])
        plt.plot(t_ref, pitch_ref * 180 / np.pi, label="ref", linestyle="--", color=color_ref)

        t_cog = np.array(data_euler_cog["__time"]) - t_bias
        pitch_cog = np.array(data_euler_cog["pitch"])
        plt.plot(t_cog, pitch_cog * 180 / np.pi, label="cog", linestyle="-.", color=color_cog)

        t = np.array(data_euler["__time"]) - t_bias
        pitch = np.array(data_euler["pitch"])
        plt.plot(t, pitch * 180 / np.pi, label="real", color=color_real)
        plt.ylabel("Pitch [$^\\circ$]", fontsize=label_size)

        if if_hand_teleop:
            plt.axvspan(con_rot_t_start, con_rot_t_stop, facecolor=matlab_green, alpha=0.2)
            plt.axvspan(
                con_rot_t_start, con_rot_t_stop, facecolor="none", edgecolor="lightgray", hatch="///", linewidth=0.0
            )

        # calculate RMSE
        rmse_pitch = calculate_rmse(t, pitch, t_ref, pitch_ref)
        print(f"RMSE Pitch [rad]: {rmse_pitch}")
        print(f"RMSE Pitch [deg]: {rmse_pitch * 180 / np.pi}")

        # --------------------------------
        plt.subplot(4, 2, 5)
        t_ref = np.array(data_xyz_ref["__time"]) - t_bias
        z_ref = np.array(data_xyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/translation/z"])
        plt.plot(t_ref, z_ref, label="ref", linestyle="--", color=color_ref)

        t_cog = np.array(data_xyz_cog["__time"]) - t_bias
        z_cog = np.array(data_xyz_cog["/beetle1/uav/cog/odom/pose/pose/position/z"])
        plt.plot(t_cog, z_cog, label="cog", linestyle="-.", color=color_cog)

        t = np.array(data_xyz["__time"]) - t_bias
        z = np.array(data_xyz["/beetle1/uav/ee_contact/odom/pose/pose/position/z"])

        plt.plot(t, z, label="Z", color=color_real)
        plt.ylabel("Z [m]", fontsize=label_size)

        if if_hand_teleop:
            plt.axvspan(inside_valve_t_start, inside_valve_t_stop, facecolor=matlab_yellow, alpha=0.2)

        # calculate RMSE
        rmse_z = calculate_rmse(t, z, t_ref, z_ref)
        print(f"RMSE Z [m]: {rmse_z}")

        # --------------------------------
        plt.subplot(4, 2, 6)
        t_ref = np.array(data_euler_ref["__time"]) - t_bias
        yaw_ref = np.array(data_euler_ref["yaw"])
        plt.plot(t_ref, yaw_ref * 180 / np.pi, label="ref", linestyle="--", color=color_ref)

        t_cog = np.array(data_euler_cog["__time"]) - t_bias
        yaw_cog = np.array(data_euler_cog["yaw"])
        plt.plot(t_cog, yaw_cog * 180 / np.pi, label="cog", linestyle="-.", color=color_cog)

        t = np.array(data_euler["__time"]) - t_bias
        yaw = np.array(data_euler["yaw"])
        plt.plot(t, yaw * 180 / np.pi, label="real", color=color_real)
        plt.ylabel("Yaw [$^\\circ$]", fontsize=label_size)

        if if_hand_teleop:
            plt.axvspan(con_rot_t_start, con_rot_t_stop, facecolor=matlab_green, alpha=0.2)
            plt.axvspan(
                con_rot_t_start, con_rot_t_stop, facecolor="none", edgecolor="lightgray", hatch="///", linewidth=0.0
            )

        # calculate RMSE
        rmse_yaw = calculate_rmse(t, yaw, t_ref, yaw_ref, is_yaw=True)
        print(f"RMSE Yaw [rad]: {rmse_yaw}")
        print(f"RMSE Yaw [deg]: {rmse_yaw * 180 / np.pi}")

        # --------------------------------
        plt.subplot(4, 2, 7)
        t = np.array(data_thrust_cmd["__time"]) - t_bias
        thrust1 = np.array(data_thrust_cmd["/beetle1/four_axes/command/base_thrust[0]"])
        plt.plot(t, thrust1, label="$f_{c1}$", linestyle="-")
        thrust2 = np.array(data_thrust_cmd["/beetle1/four_axes/command/base_thrust[1]"])
        plt.plot(t, thrust2, label="$f_{c2}$", linestyle="--")
        thrust3 = np.array(data_thrust_cmd["/beetle1/four_axes/command/base_thrust[2]"])
        plt.plot(t, thrust3, label="$f_{c3}$", linestyle="-.")
        thrust4 = np.array(data_thrust_cmd["/beetle1/four_axes/command/base_thrust[3]"])
        plt.plot(t, thrust4, label="$f_{c4}$", linestyle=":")
        plt.ylabel("Thrust Cmd [N]", fontsize=label_size)
        plt.xlabel("Time [s]", fontsize=label_size)
        plt.legend(framealpha=legend_alpha, loc="lower left", ncol=2)

        # --------------------------------
        plt.subplot(4, 2, 8)
        t = np.array(data_servo_angle_cmd["__time"]) - t_bias
        servo1 = np.array(data_servo_angle_cmd["/beetle1/gimbals_ctrl/gimbal1/position"]) * 180 / np.pi
        plt.plot(t, servo1, label="$\\alpha_{c1}$", linestyle="-")
        servo2 = np.array(data_servo_angle_cmd["/beetle1/gimbals_ctrl/gimbal2/position"]) * 180 / np.pi
        plt.plot(t, servo2, label="$\\alpha_{c2}$", linestyle="--")
        servo3 = np.array(data_servo_angle_cmd["/beetle1/gimbals_ctrl/gimbal3/position"]) * 180 / np.pi
        plt.plot(t, servo3, label="$\\alpha_{c3}$", linestyle="-.")
        servo4 = np.array(data_servo_angle_cmd["/beetle1/gimbals_ctrl/gimbal4/position"]) * 180 / np.pi
        plt.plot(t, servo4, label="$\\alpha_{c4}$", linestyle=":")

        plt.ylabel("Servo Cmd [$^\\circ$]", fontsize=label_size)
        plt.xlabel("Time [s]", fontsize=label_size)
        plt.legend(framealpha=legend_alpha, loc="center left", ncol=2)

        # --------------------------------
        plt.tight_layout()
        # make the subplots very compact
        fig.subplots_adjust(hspace=0.2)
        plt.show()

    elif type == 1:
        data_z = data[["__time", "/beetle1/uav/ee_contact/odom/pose/pose/position/z"]]
        data_z = data_z.dropna()

        data_vz = data[["__time", "/beetle1/uav/ee_contact/odom/twist/twist/linear/z"]]
        data_vz = data_vz.dropna()

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

        plt.style.use(["science", "grid"])

        plt.rcParams.update({"font.size": 11})  # default is 10
        label_size = 14

        x_min = 57
        x_max = 62.5

        fig = plt.figure(figsize=(7, 5))

        plt.subplot(2, 2, 1)
        t = np.array(data_z["__time"])
        z = np.array(data_z["/beetle1/uav/ee_contact/odom/pose/pose/position/z"])
        plt.plot(t, z, label="real")
        plt.ylabel("Z [m]", fontsize=label_size)
        plt.xlim(x_min, x_max)

        plt.subplot(2, 2, 2)
        t = np.array(data_vz["__time"])
        vz = np.array(data_vz["/beetle1/uav/ee_contact/odom/twist/twist/linear/z"])
        plt.plot(t, vz, label="real")
        plt.ylabel("Vz (m/s)", fontsize=label_size)
        plt.xlim(x_min, x_max)

        plt.subplot(2, 2, 3)
        t = np.array(data_thrust_cmd["__time"])
        thrust1 = np.array(data_thrust_cmd["/beetle1/four_axes/command/base_thrust[0]"])
        plt.plot(t, thrust1, label="$f_1$")
        thrust2 = np.array(data_thrust_cmd["/beetle1/four_axes/command/base_thrust[1]"])
        plt.plot(t, thrust2, label="$f_2$")
        thrust3 = np.array(data_thrust_cmd["/beetle1/four_axes/command/base_thrust[2]"])
        plt.plot(t, thrust3, label="$f_3$")
        thrust4 = np.array(data_thrust_cmd["/beetle1/four_axes/command/base_thrust[3]"])
        plt.plot(t, thrust4, label="$f_4$")
        plt.ylabel("Thrust Cmd [N]", fontsize=label_size)
        plt.xlabel("Time [s]", fontsize=label_size)
        plt.legend(framealpha=legend_alpha)
        plt.xlim(x_min, x_max)

        plt.subplot(2, 2, 4)
        t = np.array(data_servo_angle_cmd["__time"])
        servo1 = np.array(data_servo_angle_cmd["/beetle1/gimbals_ctrl/gimbal1/position"])
        plt.plot(t, servo1, label="$\\alpha_{c1}$")
        servo2 = np.array(data_servo_angle_cmd["/beetle1/gimbals_ctrl/gimbal2/position"])
        plt.plot(t, servo2, label="$\\alpha_{c2}$")
        servo3 = np.array(data_servo_angle_cmd["/beetle1/gimbals_ctrl/gimbal3/position"])
        plt.plot(t, servo3, label="$\\alpha_{c3}$")
        servo4 = np.array(data_servo_angle_cmd["/beetle1/gimbals_ctrl/gimbal4/position"])
        plt.plot(t, servo4, label="$\\alpha_{c4}$")
        plt.ylabel("Servo Angle Cmd [rad]", fontsize=label_size)
        plt.xlabel("Time [s]", fontsize=label_size)
        plt.legend(loc="upper right", framealpha=legend_alpha)
        plt.xlim(x_min, x_max)

        plt.tight_layout()
        plt.show()

    elif type == 2:
        plt.style.use(["science", "grid"])

        plt.rcParams.update({"font.size": 11})  # default is 10
        label_size = 14

        fig = plt.figure(figsize=(7, 4))

        t_bias = max(data_xyz["__time"].iloc[0], data_xyz_ref["__time"].iloc[0])
        color_ref = "#0C5DA5"
        color_real = "#FF2C00"

        # --------------------------------
        plt.subplot(2, 2, 1)
        t_ref = np.array(data_xyz_ref["__time"]) - t_bias
        x_ref = np.array(data_xyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/translation/x"])
        # plt.plot(t_ref, x_ref, label='X$_r$', linestyle="--")

        t = np.array(data_xyz["__time"]) - t_bias
        x = np.array(data_xyz["/beetle1/uav/ee_contact/odom/pose/pose/position/x"])
        plt.plot(t, x, label="X")

        # calculate RMSE
        rmse_x = calculate_rmse(t, x, t_ref, x_ref)
        print(f"RMSE X [m]: {rmse_x}")

        # ------
        t_ref = np.array(data_xyz_ref["__time"]) - t_bias
        y_ref = np.array(data_xyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/translation/y"])
        # plt.plot(t_ref, y_ref, label='Y$_r$', linestyle="--")

        t = np.array(data_xyz["__time"]) - t_bias
        y = np.array(data_xyz["/beetle1/uav/ee_contact/odom/pose/pose/position/y"])
        plt.plot(t, y, label="Y")

        # calculate RMSE
        rmse_y = calculate_rmse(t, y, t_ref, y_ref)
        print(f"RMSE Y [m]: {rmse_y}")

        # ------
        t_ref = np.array(data_xyz_ref["__time"]) - t_bias
        z_ref = np.array(data_xyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/translation/z"])
        plt.plot(t_ref, z_ref, label="Z$_r$", linestyle="--")

        t = np.array(data_xyz["__time"]) - t_bias
        z = np.array(data_xyz["/beetle1/uav/ee_contact/odom/pose/pose/position/z"])

        plt.plot(t, z, label="Z")

        # calculate RMSE
        rmse_z = calculate_rmse(t, z, t_ref, z_ref)
        print(f"RMSE Z [m]: {rmse_z}")

        plt.ylabel("Position [m]", fontsize=label_size)
        plt.xlim(0, 28)
        plt.legend(framealpha=legend_alpha, loc="center", ncol=2)

        # add a yellow arrow at 18.5432, -0.2201, pointing to the right
        plt.arrow(16.0, -0.2201, 2.0, 0, head_width=0.1, head_length=1.2, width=0.03, fc="y", ec="y", color="#EDB120")

        # --------------------------------
        plt.subplot(2, 2, 2)
        t_ref = np.array(data_euler_ref["__time"]) - t_bias
        roll_ref = np.array(data_euler_ref["roll"])
        # plt.plot(t_ref, roll_ref * 180 / np.pi, label='R_r', linestyle="--")

        t = np.array(data_euler["__time"]) - t_bias
        roll = np.array(data_euler["roll"])
        plt.plot(t, roll * 180 / np.pi, label="R")

        # calculate RMSE
        rmse_roll = calculate_rmse(t, roll, t_ref, roll_ref)
        print(f"RMSE Roll [rad]: {rmse_roll}")
        print(f"RMSE Roll [deg]: {rmse_roll * 180 / np.pi}")

        # ------
        t_ref = np.array(data_euler_ref["__time"]) - t_bias
        pitch_ref = np.array(data_euler_ref["pitch"])
        # plt.plot(t_ref, pitch_ref * 180 / np.pi, label='P_r', linestyle="--")

        t = np.array(data_euler["__time"]) - t_bias
        pitch = np.array(data_euler["pitch"])
        plt.plot(t, pitch * 180 / np.pi, label="P")

        # calculate RMSE
        rmse_pitch = calculate_rmse(t, pitch, t_ref, pitch_ref)
        print(f"RMSE Pitch [rad]: {rmse_pitch}")
        print(f"RMSE Pitch [deg]: {rmse_pitch * 180 / np.pi}")

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
        plt.plot(t, yaw * 180 / np.pi, label="Y")

        # calculate RMSE
        rmse_yaw = calculate_rmse(t, yaw, t_ref, yaw_ref, is_yaw=True)
        print(f"RMSE Yaw [rad]: {rmse_yaw}")
        print(f"RMSE Yaw [deg]: {rmse_yaw * 180 / np.pi}")

        plt.ylabel("Attitude [$^\\circ$]", fontsize=label_size)
        plt.xlim(0, 28)
        plt.legend(framealpha=legend_alpha, loc="upper left", ncol=3)

        # add a yellow arrow at 11.0, -22.875, pointing to the right
        plt.arrow(11.0, -22.875, 2.0, 0, head_width=5, head_length=1.2, width=1.6, fc="y", ec="y", color="b")

        # --------------------------------
        plt.subplot(2, 2, 3)
        t = np.array(data_thrust_cmd["__time"]) - t_bias
        thrust1 = np.array(data_thrust_cmd["/beetle1/four_axes/command/base_thrust[0]"])
        plt.plot(t, thrust1, label="$f_{c1}$")
        thrust2 = np.array(data_thrust_cmd["/beetle1/four_axes/command/base_thrust[1]"])
        plt.plot(t, thrust2, label="$f_{c2}$")
        thrust3 = np.array(data_thrust_cmd["/beetle1/four_axes/command/base_thrust[2]"])
        plt.plot(t, thrust3, label="$f_{c3}$")
        thrust4 = np.array(data_thrust_cmd["/beetle1/four_axes/command/base_thrust[3]"])
        plt.plot(t, thrust4, label="$f_{c4}$")
        plt.ylabel("Thrust Cmd [N]", fontsize=label_size)
        plt.xlabel("Time [s]", fontsize=label_size)
        plt.xlim(0, 28)
        plt.legend(framealpha=legend_alpha, loc="upper left", ncol=2)

        # --------------------------------
        plt.subplot(2, 2, 4)
        t = np.array(data_servo_angle_cmd["__time"]) - t_bias
        servo1 = np.array(data_servo_angle_cmd["/beetle1/gimbals_ctrl/gimbal1/position"]) * 180 / np.pi
        plt.plot(t, servo1, label="$\\alpha_{c1}$")
        servo2 = np.array(data_servo_angle_cmd["/beetle1/gimbals_ctrl/gimbal2/position"]) * 180 / np.pi
        plt.plot(t, servo2, label="$\\alpha_{c2}$")
        servo3 = np.array(data_servo_angle_cmd["/beetle1/gimbals_ctrl/gimbal3/position"]) * 180 / np.pi
        plt.plot(t, servo3, label="$\\alpha_{c3}$")
        servo4 = np.array(data_servo_angle_cmd["/beetle1/gimbals_ctrl/gimbal4/position"]) * 180 / np.pi
        plt.plot(t, servo4, label="$\\alpha_{c4}$")
        plt.ylabel("Servo Cmd [$^\\circ$]", fontsize=label_size)
        plt.xlabel("Time [s]", fontsize=label_size)
        plt.xlim(0, 28)
        plt.legend(framealpha=legend_alpha, loc="upper left", ncol=2)

        # --------------------------------
        plt.tight_layout()
        # make the subplots very compact
        fig.subplots_adjust(hspace=0.2)
        plt.show()

    elif type == 3:
        plt.style.use(["science", "grid"])

        plt.rcParams.update({"font.size": 11})  # default is 10
        label_size = 14

        fig = plt.figure(figsize=(7, 7))

        t_bias = max(data_xyz["__time"].iloc[0], data_xyz_ref["__time"].iloc[0], data_xyz_cog["__time"].iloc[0])
        color_ref = "#0C5DA5"
        color_real = "#FF2C00"
        color_cog = "#f29619"  # the orange in scienceplots

        # --------------------------------
        plt.subplot(4, 2, 1)

        t = np.array(data_xyz["__time"]) - t_bias
        x = np.array(data_xyz["/beetle1/uav/ee_contact/odom/pose/pose/position/x"])
        y = np.array(data_xyz["/beetle1/uav/ee_contact/odom/pose/pose/position/y"])
        z = np.array(data_xyz["/beetle1/uav/ee_contact/odom/pose/pose/position/z"])
        plt.plot(t, x, label="$p_x$", linestyle="-.")
        plt.plot(t, y, label="$p_y$", linestyle="--")
        plt.plot(t, z, label="$p_z$", linestyle="-")

        plt.legend(framealpha=legend_alpha)
        plt.ylabel("Position [m]", fontsize=label_size)

        # --------------------------------
        plt.subplot(4, 2, 2)

        t = np.array(data_euler["__time"]) - t_bias
        qw = np.array(data_qwxyz_interp["/beetle1/uav/ee_contact/odom/pose/pose/orientation/w"])
        qx = np.array(data_qwxyz_interp["/beetle1/uav/ee_contact/odom/pose/pose/orientation/x"])
        qy = np.array(data_qwxyz_interp["/beetle1/uav/ee_contact/odom/pose/pose/orientation/y"])
        qz = np.array(data_qwxyz_interp["/beetle1/uav/ee_contact/odom/pose/pose/orientation/z"])
        plt.plot(t, qw, label="$q_w$", linestyle=":")
        plt.plot(t, qx, label="$q_x$", linestyle="-.")
        plt.plot(t, qy, label="$q_y$", linestyle="--")
        plt.plot(t, qz, label="$q_z$", linestyle="-")

        plt.legend(framealpha=legend_alpha)
        plt.ylabel("Orientation", fontsize=label_size)

        # --------------------------------
        plt.subplot(4, 2, 3)

        t = np.array(data_iterm["__time"]) - t_bias
        fx_iterm = np.array(data_iterm["/beetle1/dist_w_f_cog_tq/iterm/wrench/force/x"])
        fy_iterm = np.array(data_iterm["/beetle1/dist_w_f_cog_tq/iterm/wrench/force/y"])
        fz_iterm = np.array(data_iterm["/beetle1/dist_w_f_cog_tq/iterm/wrench/force/z"])
        plt.plot(t, fx_iterm, label="$f_{x}$", linestyle="-.")
        plt.plot(t, fy_iterm, label="$f_{y}$", linestyle="--")
        plt.plot(t, fz_iterm, label="$f_{z}$", linestyle="-")

        plt.legend(framealpha=legend_alpha)
        plt.ylabel("${^W\\boldsymbol{f}_{dm}}$ [N]", fontsize=label_size)

        # --------------------------------
        plt.subplot(4, 2, 4)

        t = np.array(data_iterm["__time"]) - t_bias
        torque_x_iterm = np.array(data_iterm["/beetle1/dist_w_f_cog_tq/iterm/wrench/torque/x"])
        torque_y_iterm = np.array(data_iterm["/beetle1/dist_w_f_cog_tq/iterm/wrench/torque/y"])
        torque_z_iterm = np.array(data_iterm["/beetle1/dist_w_f_cog_tq/iterm/wrench/torque/z"])
        plt.plot(t, torque_x_iterm, label="$\\tau_{x}$", linestyle="-.")
        plt.plot(t, torque_y_iterm, label="$\\tau_{y}$", linestyle="--")
        plt.plot(t, torque_z_iterm, label="$\\tau_{z}$", linestyle="-")

        plt.legend(framealpha=legend_alpha)
        plt.ylabel("${^B\\boldsymbol{\\tau}_{dm}}$ [N$\cdot$m]", fontsize=label_size)

        # --------------------------------
        plt.subplot(4, 2, 5)

        t = np.array(data_ext_wrench_est["__time"]) - t_bias
        fx = np.array(data_ext_wrench_est["/beetle1/ext_wrench_est/value/wrench/force/x"])
        fy = np.array(data_ext_wrench_est["/beetle1/ext_wrench_est/value/wrench/force/y"])
        fz = np.array(data_ext_wrench_est["/beetle1/ext_wrench_est/value/wrench/force/z"])
        plt.plot(t, fx, label="$f_{x}$", linestyle="-.")
        plt.plot(t, fy, label="$f_{y}$", linestyle="--")
        plt.plot(t, fz, label="$f_{z}$", linestyle="-")

        plt.legend(framealpha=legend_alpha)
        plt.ylabel("${^B\hat{\\boldsymbol{f}}_{de,0}}$ [N]", fontsize=label_size)

        # --------------------------------
        plt.subplot(4, 2, 6)

        t = np.array(data_ext_wrench_est["__time"]) - t_bias
        torque_x = np.array(data_ext_wrench_est["/beetle1/ext_wrench_est/value/wrench/torque/x"])
        torque_y = np.array(data_ext_wrench_est["/beetle1/ext_wrench_est/value/wrench/torque/y"])
        torque_z = np.array(data_ext_wrench_est["/beetle1/ext_wrench_est/value/wrench/torque/z"])
        plt.plot(t, torque_x, label="$\\tau_{x}$", linestyle="-.")
        plt.plot(t, torque_y, label="$\\tau_{y}$", linestyle="--")
        plt.plot(t, torque_z, label="$\\tau_{z}$", linestyle="-")

        plt.legend(framealpha=legend_alpha)
        plt.ylabel("${^B\hat{\\boldsymbol{\\tau}}_{de,0}}$ [N$\cdot$m]", fontsize=label_size)

        # --------------------------------
        plt.subplot(4, 2, 7)
        t = np.array(data_thrust_cmd["__time"]) - t_bias
        thrust1 = np.array(data_thrust_cmd["/beetle1/four_axes/command/base_thrust[0]"])
        plt.plot(t, thrust1, label="$f_{c1}$", linestyle="-")
        thrust2 = np.array(data_thrust_cmd["/beetle1/four_axes/command/base_thrust[1]"])
        plt.plot(t, thrust2, label="$f_{c2}$", linestyle="--")
        thrust3 = np.array(data_thrust_cmd["/beetle1/four_axes/command/base_thrust[2]"])
        plt.plot(t, thrust3, label="$f_{c3}$", linestyle="-.")
        thrust4 = np.array(data_thrust_cmd["/beetle1/four_axes/command/base_thrust[3]"])
        plt.plot(t, thrust4, label="$f_{c4}$", linestyle=":")
        plt.ylabel("Thrust Cmd [N]", fontsize=label_size)
        plt.xlabel("Time [s]", fontsize=label_size)
        plt.legend(framealpha=legend_alpha, loc="upper center", ncol=2)

        # --------------------------------
        plt.subplot(4, 2, 8)
        t = np.array(data_servo_angle_cmd["__time"]) - t_bias
        servo1 = np.array(data_servo_angle_cmd["/beetle1/gimbals_ctrl/gimbal1/position"]) * 180 / np.pi
        plt.plot(t, servo1, label="$\\alpha_{c1}$", linestyle="-")
        servo2 = np.array(data_servo_angle_cmd["/beetle1/gimbals_ctrl/gimbal2/position"]) * 180 / np.pi
        plt.plot(t, servo2, label="$\\alpha_{c2}$", linestyle="--")
        servo3 = np.array(data_servo_angle_cmd["/beetle1/gimbals_ctrl/gimbal3/position"]) * 180 / np.pi
        plt.plot(t, servo3, label="$\\alpha_{c3}$", linestyle="-.")
        servo4 = np.array(data_servo_angle_cmd["/beetle1/gimbals_ctrl/gimbal4/position"]) * 180 / np.pi
        plt.plot(t, servo4, label="$\\alpha_{c4}$", linestyle=":")

        plt.ylabel("Servo Cmd [$^\\circ$]", fontsize=label_size)
        plt.xlabel("Time [s]", fontsize=label_size)
        plt.legend(framealpha=legend_alpha, loc="center", ncol=2)

        # --------------------------------
        plt.tight_layout()
        # make the subplots very compact
        fig.subplots_adjust(hspace=0.2)
        plt.show()

    else:
        print("Invalid type")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Plot the trajectory. Please use plotjuggler to generate the csv file."
    )
    parser.add_argument("file_path", type=str, help="The file name of the trajectory")
    parser.add_argument("-t", "--type", type=int, help="The type of the trajectory")
    parser.add_argument("-o", "--hand_teleop", action="store_true", help="Whether the trajectory is from hand teleop")

    args = parser.parse_args()

    main(args.file_path, args.type, args.hand_teleop)
