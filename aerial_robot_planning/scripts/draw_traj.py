import pandas as pd
import numpy as np
import scienceplots
import matplotlib.pyplot as plt
import argparse

legend_alpha = 0.5


def calculate_rmse(t, x, t_ref, x_ref, is_yaw=False):
    x_ref_interp = np.interp(t, t_ref, x_ref)
    if is_yaw:
        # calculate the RMSE for yaw
        error = np.minimum(np.abs(x - x_ref_interp), 2 * np.pi - np.abs(x - x_ref_interp))
    else:
        error = x - x_ref_interp

    rmse_x = np.sqrt(np.mean(error ** 2))
    return rmse_x


def main(file_path, type):
    # Load the data from csv file
    data = pd.read_csv(file_path)

    # ======= xyz =========
    data_xyz = data[
        ['__time', '/beetle1/uav/cog/odom/pose/pose/position/x', '/beetle1/uav/cog/odom/pose/pose/position/y',
         '/beetle1/uav/cog/odom/pose/pose/position/z']]

    try:
        data_xyz_ref = data[
            ['__time', '/beetle1/set_ref_traj/x/data[0]', '/beetle1/set_ref_traj/x/data[1]',
             '/beetle1/set_ref_traj/x/data[2]']]
    except KeyError:
        # assign the reference trajectory to zero
        data_xyz_ref = pd.DataFrame()
        data_xyz_ref['__time'] = data_xyz['__time']
        data_xyz_ref['/beetle1/set_ref_traj/x/data[0]'] = -0.095
        data_xyz_ref['/beetle1/set_ref_traj/x/data[1]'] = -0.015
        data_xyz_ref['/beetle1/set_ref_traj/x/data[2]'] = 0.6

    data_xyz = data_xyz.dropna()
    data_xyz_ref = data_xyz_ref.dropna()

    # ======= rpy =========
    data_qwxyz = data[
        ['__time', '/beetle1/uav/cog/odom/pose/pose/orientation/w', '/beetle1/uav/cog/odom/pose/pose/orientation/x',
         '/beetle1/uav/cog/odom/pose/pose/orientation/y', '/beetle1/uav/cog/odom/pose/pose/orientation/z']]

    try:
        data_qwxyz_ref = data[
            ['__time', '/beetle1/set_ref_traj/x/data[6]', '/beetle1/set_ref_traj/x/data[7]',
             '/beetle1/set_ref_traj/x/data[8]',
             '/beetle1/set_ref_traj/x/data[9]']]
    except KeyError:
        # assign the reference trajectory to zero
        data_qwxyz_ref = pd.DataFrame()
        data_qwxyz_ref['__time'] = data_qwxyz['__time']
        data_qwxyz_ref['/beetle1/set_ref_traj/x/data[6]'] = 0
        data_qwxyz_ref['/beetle1/set_ref_traj/x/data[7]'] = 0
        data_qwxyz_ref['/beetle1/set_ref_traj/x/data[8]'] = 0
        data_qwxyz_ref['/beetle1/set_ref_traj/x/data[9]'] = 0

    data_qwxyz_ref = data_qwxyz_ref.dropna()
    data_qwxyz = data_qwxyz.dropna()

    # convert to euler
    data_euler_ref = pd.DataFrame()
    data_euler_ref['__time'] = data_qwxyz_ref['__time']
    data_euler_ref['roll'] = np.arctan2(2 * (data_qwxyz_ref['/beetle1/set_ref_traj/x/data[9]'] *
                                             data_qwxyz_ref['/beetle1/set_ref_traj/x/data[8]'] +
                                             data_qwxyz_ref['/beetle1/set_ref_traj/x/data[6]'] *
                                             data_qwxyz_ref['/beetle1/set_ref_traj/x/data[7]']),
                                        1 - 2 * (data_qwxyz_ref['/beetle1/set_ref_traj/x/data[7]'] ** 2 +
                                                 data_qwxyz_ref['/beetle1/set_ref_traj/x/data[8]'] ** 2))
    data_euler_ref['pitch'] = np.arcsin(2 * (data_qwxyz_ref['/beetle1/set_ref_traj/x/data[6]'] *
                                             data_qwxyz_ref['/beetle1/set_ref_traj/x/data[8]'] -
                                             data_qwxyz_ref['/beetle1/set_ref_traj/x/data[9]'] *
                                             data_qwxyz_ref['/beetle1/set_ref_traj/x/data[7]']))
    data_euler_ref['yaw'] = np.arctan2(2 * (data_qwxyz_ref['/beetle1/set_ref_traj/x/data[6]'] *
                                            data_qwxyz_ref['/beetle1/set_ref_traj/x/data[9]'] +
                                            data_qwxyz_ref['/beetle1/set_ref_traj/x/data[7]'] *
                                            data_qwxyz_ref['/beetle1/set_ref_traj/x/data[8]']),
                                       1 - 2 * (data_qwxyz_ref['/beetle1/set_ref_traj/x/data[7]'] ** 2 +
                                                data_qwxyz_ref['/beetle1/set_ref_traj/x/data[9]'] ** 2))

    data_euler = pd.DataFrame()
    data_euler['__time'] = data_qwxyz['__time']
    data_euler['roll'] = np.arctan2(2 * (data_qwxyz['/beetle1/uav/cog/odom/pose/pose/orientation/z'] *
                                         data_qwxyz['/beetle1/uav/cog/odom/pose/pose/orientation/y'] +
                                         data_qwxyz['/beetle1/uav/cog/odom/pose/pose/orientation/w'] *
                                         data_qwxyz['/beetle1/uav/cog/odom/pose/pose/orientation/x']),
                                    1 - 2 * (data_qwxyz['/beetle1/uav/cog/odom/pose/pose/orientation/x'] ** 2 +
                                             data_qwxyz['/beetle1/uav/cog/odom/pose/pose/orientation/y'] ** 2))
    data_euler['pitch'] = np.arcsin(2 * (data_qwxyz['/beetle1/uav/cog/odom/pose/pose/orientation/w'] *
                                         data_qwxyz['/beetle1/uav/cog/odom/pose/pose/orientation/y'] -
                                         data_qwxyz['/beetle1/uav/cog/odom/pose/pose/orientation/z'] *
                                         data_qwxyz['/beetle1/uav/cog/odom/pose/pose/orientation/x']))
    data_euler['yaw'] = np.arctan2(2 * (data_qwxyz['/beetle1/uav/cog/odom/pose/pose/orientation/w'] *
                                        data_qwxyz['/beetle1/uav/cog/odom/pose/pose/orientation/z'] +
                                        data_qwxyz['/beetle1/uav/cog/odom/pose/pose/orientation/x'] *
                                        data_qwxyz['/beetle1/uav/cog/odom/pose/pose/orientation/y']),
                                   1 - 2 * (data_qwxyz['/beetle1/uav/cog/odom/pose/pose/orientation/y'] ** 2 +
                                            data_qwxyz['/beetle1/uav/cog/odom/pose/pose/orientation/z'] ** 2))

    # thrust_cmd
    data_thrust_cmd = data[
        ['__time', '/beetle1/four_axes/command/base_thrust[0]', '/beetle1/four_axes/command/base_thrust[1]',
         '/beetle1/four_axes/command/base_thrust[2]', '/beetle1/four_axes/command/base_thrust[3]']]
    data_thrust_cmd = data_thrust_cmd.dropna()

    # servo angle cmd
    data_servo_angle_cmd = data[
        ['__time', '/beetle1/gimbals_ctrl/gimbal1/position', '/beetle1/gimbals_ctrl/gimbal2/position',
         '/beetle1/gimbals_ctrl/gimbal3/position', '/beetle1/gimbals_ctrl/gimbal4/position']]
    data_servo_angle_cmd = data_servo_angle_cmd.dropna()

    # ======= plotting =========
    if type == 0:
        plt.style.use(["science", "grid"])

        plt.rcParams.update({'font.size': 11})  # default is 10
        label_size = 14

        fig = plt.figure(figsize=(7, 7))

        t_bias = max(data_xyz['__time'].iloc[0], data_xyz_ref['__time'].iloc[0])
        color_ref = '#0C5DA5'
        color_real = '#FF2C00'

        # --------------------------------
        plt.subplot(4, 2, 1)
        t_ref = np.array(data_xyz_ref['__time']) - t_bias
        x_ref = np.array(data_xyz_ref['/beetle1/set_ref_traj/x/data[0]'])
        plt.plot(t_ref, x_ref, label='ref', linestyle="--", color=color_ref)

        t = np.array(data_xyz['__time']) - t_bias
        x = np.array(data_xyz['/beetle1/uav/cog/odom/pose/pose/position/x'])
        plt.plot(t, x, label='real', color=color_real)

        plt.legend(framealpha=legend_alpha)
        plt.ylabel('X (m)', fontsize=label_size)

        # calculate RMSE
        rmse_x = calculate_rmse(t, x, t_ref, x_ref)
        print(f'RMSE X (m): {rmse_x}')

        # --------------------------------
        plt.subplot(4, 2, 2)
        t_ref = np.array(data_euler_ref['__time']) - t_bias
        roll_ref = np.array(data_euler_ref['roll'])
        plt.plot(t_ref, roll_ref * 180 / np.pi, label='ref', linestyle="--", color=color_ref)

        t = np.array(data_euler['__time']) - t_bias
        roll = np.array(data_euler['roll'])
        plt.plot(t, roll * 180 / np.pi, label='real', color=color_real)

        plt.ylabel('Roll ($^\\circ$)', fontsize=label_size)

        # calculate RMSE
        rmse_roll = calculate_rmse(t, roll, t_ref, roll_ref)
        print(f'RMSE Roll (rad): {rmse_roll}')
        print(f'RMSE Roll (deg): {rmse_roll * 180 / np.pi}')

        # --------------------------------
        plt.subplot(4, 2, 3)
        t_ref = np.array(data_xyz_ref['__time']) - t_bias
        y_ref = np.array(data_xyz_ref['/beetle1/set_ref_traj/x/data[1]'])
        plt.plot(t_ref, y_ref, label='ref', linestyle="--", color=color_ref)

        t = np.array(data_xyz['__time']) - t_bias
        y = np.array(data_xyz['/beetle1/uav/cog/odom/pose/pose/position/y'])
        plt.plot(t, y, label='Y', color=color_real)
        plt.ylabel('Y (m)', fontsize=label_size)

        # calculate RMSE
        rmse_y = calculate_rmse(t, y, t_ref, y_ref)
        print(f'RMSE Y (m): {rmse_y}')

        # --------------------------------
        plt.subplot(4, 2, 4)
        t_ref = np.array(data_euler_ref['__time']) - t_bias
        pitch_ref = np.array(data_euler_ref['pitch'])
        plt.plot(t_ref, pitch_ref * 180 / np.pi, label='ref', linestyle="--", color=color_ref)

        t = np.array(data_euler['__time']) - t_bias
        pitch = np.array(data_euler['pitch'])
        plt.plot(t, pitch * 180 / np.pi, label='real', color=color_real)
        plt.ylabel('Pitch ($^\\circ$)', fontsize=label_size)

        # calculate RMSE
        rmse_pitch = calculate_rmse(t, pitch, t_ref, pitch_ref)
        print(f'RMSE Pitch (rad): {rmse_pitch}')
        print(f'RMSE Pitch (deg): {rmse_pitch * 180 / np.pi}')

        # --------------------------------
        plt.subplot(4, 2, 5)
        t_ref = np.array(data_xyz_ref['__time']) - t_bias
        z_ref = np.array(data_xyz_ref['/beetle1/set_ref_traj/x/data[2]'])
        plt.plot(t_ref, z_ref, label='ref', linestyle="--", color=color_ref)

        t = np.array(data_xyz['__time']) - t_bias
        z = np.array(data_xyz['/beetle1/uav/cog/odom/pose/pose/position/z'])

        plt.plot(t, z, label='Z', color=color_real)
        plt.ylabel('Z (m)', fontsize=label_size)

        # calculate RMSE
        rmse_z = calculate_rmse(t, z, t_ref, z_ref)
        print(f'RMSE Z (m): {rmse_z}')

        # --------------------------------
        plt.subplot(4, 2, 6)
        t_ref = np.array(data_euler_ref['__time']) - t_bias
        yaw_ref = np.array(data_euler_ref['yaw'])
        # if yaw_ref has a jump, we need to fix it
        for i in range(1, len(yaw_ref)):
            if yaw_ref[i] - yaw_ref[i - 1] > np.pi:
                yaw_ref[i:] -= 2 * np.pi
            elif yaw_ref[i] - yaw_ref[i - 1] < -np.pi:
                yaw_ref[i:] += 2 * np.pi
        plt.plot(t_ref, yaw_ref * 180 / np.pi, label='ref', linestyle="--", color=color_ref)

        t = np.array(data_euler['__time']) - t_bias
        yaw = np.array(data_euler['yaw'])
        # if yaw has a jump, we need to fix it
        for i in range(1, len(yaw)):
            if yaw[i] - yaw[i - 1] > np.pi:
                yaw[i:] -= 2 * np.pi
            elif yaw[i] - yaw[i - 1] < -np.pi:
                yaw[i:] += 2 * np.pi
        plt.plot(t, yaw * 180 / np.pi, label='real', color=color_real)
        plt.ylabel('Yaw ($^\\circ$)', fontsize=label_size)

        # calculate RMSE
        rmse_yaw = calculate_rmse(t, yaw, t_ref, yaw_ref, is_yaw=True)
        print(f'RMSE Yaw (rad): {rmse_yaw}')
        print(f'RMSE Yaw (deg): {rmse_yaw * 180 / np.pi}')

        # --------------------------------
        plt.subplot(4, 2, 7)
        t = np.array(data_thrust_cmd['__time']) - t_bias
        thrust1 = np.array(data_thrust_cmd['/beetle1/four_axes/command/base_thrust[0]'])
        plt.plot(t, thrust1, label='$f_{c1}$')
        thrust2 = np.array(data_thrust_cmd['/beetle1/four_axes/command/base_thrust[1]'])
        plt.plot(t, thrust2, label='$f_{c2}$')
        thrust3 = np.array(data_thrust_cmd['/beetle1/four_axes/command/base_thrust[2]'])
        plt.plot(t, thrust3, label='$f_{c3}$')
        thrust4 = np.array(data_thrust_cmd['/beetle1/four_axes/command/base_thrust[3]'])
        plt.plot(t, thrust4, label='$f_{c4}$')
        plt.ylabel('Thrust Cmd (N)', fontsize=label_size)
        plt.xlabel('Time (s)', fontsize=label_size)
        plt.legend(framealpha=legend_alpha, loc='upper left')

        # --------------------------------
        plt.subplot(4, 2, 8)
        t = np.array(data_servo_angle_cmd['__time']) - t_bias
        servo1 = np.array(data_servo_angle_cmd['/beetle1/gimbals_ctrl/gimbal1/position']) * 180 / np.pi
        plt.plot(t, servo1, label='$\\alpha_{c1}$')
        servo2 = np.array(data_servo_angle_cmd['/beetle1/gimbals_ctrl/gimbal2/position']) * 180 / np.pi
        plt.plot(t, servo2, label='$\\alpha_{c2}$')
        servo3 = np.array(data_servo_angle_cmd['/beetle1/gimbals_ctrl/gimbal3/position']) * 180 / np.pi
        plt.plot(t, servo3, label='$\\alpha_{c3}$')
        servo4 = np.array(data_servo_angle_cmd['/beetle1/gimbals_ctrl/gimbal4/position']) * 180 / np.pi
        plt.plot(t, servo4, label='$\\alpha_{c4}$')
        plt.ylabel('Servo Cmd ($^\\circ$)', fontsize=label_size)
        plt.xlabel('Time (s)', fontsize=label_size)
        plt.legend(framealpha=legend_alpha, loc='upper left')

        # --------------------------------
        plt.tight_layout()
        # make the subplots very compact
        fig.subplots_adjust(hspace=0.2)
        plt.show()

    elif type == 1:
        data_z = data[['__time', '/beetle1/uav/cog/odom/pose/pose/position/z']]
        data_z = data_z.dropna()

        data_vz = data[['__time', '/beetle1/uav/cog/odom/twist/twist/linear/z']]
        data_vz = data_vz.dropna()

        data_thrust_cmd = data[
            ['__time', '/beetle1/four_axes/command/base_thrust[0]', '/beetle1/four_axes/command/base_thrust[1]',
             '/beetle1/four_axes/command/base_thrust[2]', '/beetle1/four_axes/command/base_thrust[3]']]
        data_thrust_cmd = data_thrust_cmd.dropna()

        data_servo_angle_cmd = data[
            ['__time', '/beetle1/gimbals_ctrl/gimbal1/position', '/beetle1/gimbals_ctrl/gimbal2/position',
             '/beetle1/gimbals_ctrl/gimbal3/position', '/beetle1/gimbals_ctrl/gimbal4/position']]
        data_servo_angle_cmd = data_servo_angle_cmd.dropna()

        plt.style.use(["science", "grid"])

        plt.rcParams.update({'font.size': 11})  # default is 10
        label_size = 14

        x_min = 57
        x_max = 62.5

        fig = plt.figure(figsize=(7, 5))

        plt.subplot(2, 2, 1)
        t = np.array(data_z['__time'])
        z = np.array(data_z['/beetle1/uav/cog/odom/pose/pose/position/z'])
        plt.plot(t, z, label='real')
        plt.ylabel('Z (m)', fontsize=label_size)
        plt.xlim(x_min, x_max)

        plt.subplot(2, 2, 2)
        t = np.array(data_vz['__time'])
        vz = np.array(data_vz['/beetle1/uav/cog/odom/twist/twist/linear/z'])
        plt.plot(t, vz, label='real')
        plt.ylabel('Vz (m/s)', fontsize=label_size)
        plt.xlim(x_min, x_max)

        plt.subplot(2, 2, 3)
        t = np.array(data_thrust_cmd['__time'])
        thrust1 = np.array(data_thrust_cmd['/beetle1/four_axes/command/base_thrust[0]'])
        plt.plot(t, thrust1, label='$f_1$')
        thrust2 = np.array(data_thrust_cmd['/beetle1/four_axes/command/base_thrust[1]'])
        plt.plot(t, thrust2, label='$f_2$')
        thrust3 = np.array(data_thrust_cmd['/beetle1/four_axes/command/base_thrust[2]'])
        plt.plot(t, thrust3, label='$f_3$')
        thrust4 = np.array(data_thrust_cmd['/beetle1/four_axes/command/base_thrust[3]'])
        plt.plot(t, thrust4, label='$f_4$')
        plt.ylabel('Thrust Cmd (N)', fontsize=label_size)
        plt.xlabel('Time (s)', fontsize=label_size)
        plt.legend(framealpha=legend_alpha)
        plt.xlim(x_min, x_max)

        plt.subplot(2, 2, 4)
        t = np.array(data_servo_angle_cmd['__time'])
        servo1 = np.array(data_servo_angle_cmd['/beetle1/gimbals_ctrl/gimbal1/position'])
        plt.plot(t, servo1, label='$\\alpha_{c1}$')
        servo2 = np.array(data_servo_angle_cmd['/beetle1/gimbals_ctrl/gimbal2/position'])
        plt.plot(t, servo2, label='$\\alpha_{c2}$')
        servo3 = np.array(data_servo_angle_cmd['/beetle1/gimbals_ctrl/gimbal3/position'])
        plt.plot(t, servo3, label='$\\alpha_{c3}$')
        servo4 = np.array(data_servo_angle_cmd['/beetle1/gimbals_ctrl/gimbal4/position'])
        plt.plot(t, servo4, label='$\\alpha_{c4}$')
        plt.ylabel('Servo Angle Cmd (rad)', fontsize=label_size)
        plt.xlabel('Time (s)', fontsize=label_size)
        plt.legend(loc='upper right', framealpha=legend_alpha)
        plt.xlim(x_min, x_max)

        plt.tight_layout()
        plt.show()

    elif type == 2:
        plt.style.use(["science", "grid"])

        plt.rcParams.update({'font.size': 11})  # default is 10
        label_size = 14

        fig = plt.figure(figsize=(7, 4))

        t_bias = max(data_xyz['__time'].iloc[0], data_xyz_ref['__time'].iloc[0])
        color_ref = '#0C5DA5'
        color_real = '#FF2C00'

        # --------------------------------
        plt.subplot(2, 2, 1)
        t_ref = np.array(data_xyz_ref['__time']) - t_bias
        x_ref = np.array(data_xyz_ref['/beetle1/set_ref_traj/x/data[0]'])
        # plt.plot(t_ref, x_ref, label='X$_r$', linestyle="--")

        t = np.array(data_xyz['__time']) - t_bias
        x = np.array(data_xyz['/beetle1/uav/cog/odom/pose/pose/position/x'])
        plt.plot(t, x, label='X')

        # calculate RMSE
        rmse_x = calculate_rmse(t, x, t_ref, x_ref)
        print(f'RMSE X (m): {rmse_x}')

        # ------
        t_ref = np.array(data_xyz_ref['__time']) - t_bias
        y_ref = np.array(data_xyz_ref['/beetle1/set_ref_traj/x/data[1]'])
        # plt.plot(t_ref, y_ref, label='Y$_r$', linestyle="--")

        t = np.array(data_xyz['__time']) - t_bias
        y = np.array(data_xyz['/beetle1/uav/cog/odom/pose/pose/position/y'])
        plt.plot(t, y, label='Y')

        # calculate RMSE
        rmse_y = calculate_rmse(t, y, t_ref, y_ref)
        print(f'RMSE Y (m): {rmse_y}')

        # ------
        t_ref = np.array(data_xyz_ref['__time']) - t_bias
        z_ref = np.array(data_xyz_ref['/beetle1/set_ref_traj/x/data[2]'])
        plt.plot(t_ref, z_ref, label='Z$_r$', linestyle="--")

        t = np.array(data_xyz['__time']) - t_bias
        z = np.array(data_xyz['/beetle1/uav/cog/odom/pose/pose/position/z'])

        plt.plot(t, z, label='Z')

        # calculate RMSE
        rmse_z = calculate_rmse(t, z, t_ref, z_ref)
        print(f'RMSE Z (m): {rmse_z}')

        plt.ylabel('Position (m)', fontsize=label_size)
        plt.xlim(0, 28)
        plt.legend(framealpha=legend_alpha, loc='center', ncol=2)

        # add a yellow arrow at 18.5432, -0.2201, pointing to the right
        plt.arrow(16.0, -0.2201, 2.0, 0, head_width=0.1, head_length=1.2, width=0.03, fc='y', ec='y', color="#EDB120")

        # --------------------------------
        plt.subplot(2, 2, 2)
        t_ref = np.array(data_euler_ref['__time']) - t_bias
        roll_ref = np.array(data_euler_ref['roll'])
        # plt.plot(t_ref, roll_ref * 180 / np.pi, label='R_r', linestyle="--")

        t = np.array(data_euler['__time']) - t_bias
        roll = np.array(data_euler['roll'])
        plt.plot(t, roll * 180 / np.pi, label='R')

        # calculate RMSE
        rmse_roll = calculate_rmse(t, roll, t_ref, roll_ref)
        print(f'RMSE Roll (rad): {rmse_roll}')
        print(f'RMSE Roll (deg): {rmse_roll * 180 / np.pi}')

        # ------
        t_ref = np.array(data_euler_ref['__time']) - t_bias
        pitch_ref = np.array(data_euler_ref['pitch'])
        # plt.plot(t_ref, pitch_ref * 180 / np.pi, label='P_r', linestyle="--")

        t = np.array(data_euler['__time']) - t_bias
        pitch = np.array(data_euler['pitch'])
        plt.plot(t, pitch * 180 / np.pi, label='P')

        # calculate RMSE
        rmse_pitch = calculate_rmse(t, pitch, t_ref, pitch_ref)
        print(f'RMSE Pitch (rad): {rmse_pitch}')
        print(f'RMSE Pitch (deg): {rmse_pitch * 180 / np.pi}')

        # ------
        t_ref = np.array(data_euler_ref['__time']) - t_bias
        yaw_ref = np.array(data_euler_ref['yaw'])
        # if yaw_ref has a jump, we need to fix it
        for i in range(1, len(yaw_ref)):
            if yaw_ref[i] - yaw_ref[i - 1] > np.pi:
                yaw_ref[i:] -= 2 * np.pi
            elif yaw_ref[i] - yaw_ref[i - 1] < -np.pi:
                yaw_ref[i:] += 2 * np.pi
        # plt.plot(t_ref, yaw_ref * 180 / np.pi, label='Y_r', linestyle="--")

        t = np.array(data_euler['__time']) - t_bias
        yaw = np.array(data_euler['yaw'])
        # if yaw has a jump, we need to fix it
        for i in range(1, len(yaw)):
            if yaw[i] - yaw[i - 1] > np.pi:
                yaw[i:] -= 2 * np.pi
            elif yaw[i] - yaw[i - 1] < -np.pi:
                yaw[i:] += 2 * np.pi
        plt.plot(t, yaw * 180 / np.pi, label='Y')

        # calculate RMSE
        rmse_yaw = calculate_rmse(t, yaw, t_ref, yaw_ref, is_yaw=True)
        print(f'RMSE Yaw (rad): {rmse_yaw}')
        print(f'RMSE Yaw (deg): {rmse_yaw * 180 / np.pi}')

        plt.ylabel('Attitude ($^\\circ$)', fontsize=label_size)
        plt.xlim(0, 28)
        plt.legend(framealpha=legend_alpha, loc='upper left', ncol=3)

        # add a yellow arrow at 11.0, -22.875, pointing to the right
        plt.arrow(11.0, -22.875, 2.0, 0, head_width=5, head_length=1.2, width=1.6, fc='y', ec='y', color="b")

        # --------------------------------
        plt.subplot(2, 2, 3)
        t = np.array(data_thrust_cmd['__time']) - t_bias
        thrust1 = np.array(data_thrust_cmd['/beetle1/four_axes/command/base_thrust[0]'])
        plt.plot(t, thrust1, label='$f_{c1}$')
        thrust2 = np.array(data_thrust_cmd['/beetle1/four_axes/command/base_thrust[1]'])
        plt.plot(t, thrust2, label='$f_{c2}$')
        thrust3 = np.array(data_thrust_cmd['/beetle1/four_axes/command/base_thrust[2]'])
        plt.plot(t, thrust3, label='$f_{c3}$')
        thrust4 = np.array(data_thrust_cmd['/beetle1/four_axes/command/base_thrust[3]'])
        plt.plot(t, thrust4, label='$f_{c4}$')
        plt.ylabel('Thrust Cmd (N)', fontsize=label_size)
        plt.xlabel('Time (s)', fontsize=label_size)
        plt.xlim(0, 28)
        plt.legend(framealpha=legend_alpha, loc='upper left', ncol=2)

        # --------------------------------
        plt.subplot(2, 2, 4)
        t = np.array(data_servo_angle_cmd['__time']) - t_bias
        servo1 = np.array(data_servo_angle_cmd['/beetle1/gimbals_ctrl/gimbal1/position']) * 180 / np.pi
        plt.plot(t, servo1, label='$\\alpha_{c1}$')
        servo2 = np.array(data_servo_angle_cmd['/beetle1/gimbals_ctrl/gimbal2/position']) * 180 / np.pi
        plt.plot(t, servo2, label='$\\alpha_{c2}$')
        servo3 = np.array(data_servo_angle_cmd['/beetle1/gimbals_ctrl/gimbal3/position']) * 180 / np.pi
        plt.plot(t, servo3, label='$\\alpha_{c3}$')
        servo4 = np.array(data_servo_angle_cmd['/beetle1/gimbals_ctrl/gimbal4/position']) * 180 / np.pi
        plt.plot(t, servo4, label='$\\alpha_{c4}$')
        plt.ylabel('Servo Cmd ($^\\circ$)', fontsize=label_size)
        plt.xlabel('Time (s)', fontsize=label_size)
        plt.xlim(0, 28)
        plt.legend(framealpha=legend_alpha, loc='upper left', ncol=2)

        # --------------------------------
        plt.tight_layout()
        # make the subplots very compact
        fig.subplots_adjust(hspace=0.2)
        plt.show()

    else:
        print('Invalid type')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Plot the trajectory. Please use plotjuggler to generate the csv file.')
    parser.add_argument('file_path', type=str, help='The file name of the trajectory')
    parser.add_argument('--type', type=int, help='The type of the trajectory')

    args = parser.parse_args()

    main(args.file_path, args.type)
