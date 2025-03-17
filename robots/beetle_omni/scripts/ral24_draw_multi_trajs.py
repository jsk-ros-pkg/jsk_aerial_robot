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


def quat2euler(qw, qx, qy, qz):
    roll = np.arctan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx ** 2 + qy ** 2))
    pitch = np.arcsin(2 * (qw * qy - qz * qx))
    yaw = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy ** 2 + qz ** 2))
    return roll, pitch, yaw

def main(file_path_1, file_path_2):
    # Load the data from csv file
    data_1 = pd.read_csv(file_path_1)
    data_2 = pd.read_csv(file_path_2)

    # ======= xyz =========
    data_xyz_1 = data_1[
        ['__time', '/beetle1/uav/cog/odom/pose/pose/position/x', '/beetle1/uav/cog/odom/pose/pose/position/y',
         '/beetle1/uav/cog/odom/pose/pose/position/z']]
    data_xyz_2 = data_2[
        ['__time', '/beetle1/uav/cog/odom/pose/pose/position/x', '/beetle1/uav/cog/odom/pose/pose/position/y',
         '/beetle1/uav/cog/odom/pose/pose/position/z']]

    data_xyz_ref_1 = data_1[
        ['__time', '/beetle1/set_ref_traj/x/data[0]', '/beetle1/set_ref_traj/x/data[1]',
         '/beetle1/set_ref_traj/x/data[2]']]
    data_xyz_ref_2 = data_2[
        ['__time', '/beetle1/set_ref_traj/x/data[0]', '/beetle1/set_ref_traj/x/data[1]',
         '/beetle1/set_ref_traj/x/data[2]']]

    data_xyz_1 = data_xyz_1.dropna()
    data_xyz_2 = data_xyz_2.dropna()
    data_xyz_ref_1 = data_xyz_ref_1.dropna()
    data_xyz_ref_2 = data_xyz_ref_2.dropna()

    # ======= rpy =========
    data_qwxyz_1 = data_1[
        ['__time', '/beetle1/uav/cog/odom/pose/pose/orientation/w', '/beetle1/uav/cog/odom/pose/pose/orientation/x',
         '/beetle1/uav/cog/odom/pose/pose/orientation/y', '/beetle1/uav/cog/odom/pose/pose/orientation/z']]
    data_qwxyz_2 = data_2[
        ['__time', '/beetle1/uav/cog/odom/pose/pose/orientation/w', '/beetle1/uav/cog/odom/pose/pose/orientation/x',
         '/beetle1/uav/cog/odom/pose/pose/orientation/y', '/beetle1/uav/cog/odom/pose/pose/orientation/z']]

    data_qwxyz_ref_1 = data_1[
        ['__time', '/beetle1/set_ref_traj/x/data[6]', '/beetle1/set_ref_traj/x/data[7]',
         '/beetle1/set_ref_traj/x/data[8]',
         '/beetle1/set_ref_traj/x/data[9]']]
    data_qwxyz_ref_2 = data_2[
        ['__time', '/beetle1/set_ref_traj/x/data[6]', '/beetle1/set_ref_traj/x/data[7]',
         '/beetle1/set_ref_traj/x/data[8]',
         '/beetle1/set_ref_traj/x/data[9]']]

    data_qwxyz_ref_1 = data_qwxyz_ref_1.dropna()
    data_qwxyz_ref_2 = data_qwxyz_ref_2.dropna()
    data_qwxyz_1 = data_qwxyz_1.dropna()
    data_qwxyz_2 = data_qwxyz_2.dropna()

    # convert to euler
    data_euler_ref_1 = pd.DataFrame()
    data_euler_ref_1['__time'] = data_qwxyz_ref_1['__time']
    data_euler_ref_1['roll'], data_euler_ref_1['pitch'], data_euler_ref_1['yaw'] = quat2euler(
        data_qwxyz_ref_1['/beetle1/set_ref_traj/x/data[6]'], data_qwxyz_ref_1['/beetle1/set_ref_traj/x/data[7]'],
        data_qwxyz_ref_1['/beetle1/set_ref_traj/x/data[8]'], data_qwxyz_ref_1['/beetle1/set_ref_traj/x/data[9]'])

    data_euler_ref_2 = pd.DataFrame()
    data_euler_ref_2['__time'] = data_qwxyz_ref_2['__time']
    data_euler_ref_2['roll'], data_euler_ref_2['pitch'], data_euler_ref_2['yaw'] = quat2euler(
        data_qwxyz_ref_2['/beetle1/set_ref_traj/x/data[6]'], data_qwxyz_ref_2['/beetle1/set_ref_traj/x/data[7]'],
        data_qwxyz_ref_2['/beetle1/set_ref_traj/x/data[8]'], data_qwxyz_ref_2['/beetle1/set_ref_traj/x/data[9]'])

    data_euler_1 = pd.DataFrame()
    data_euler_1['__time'] = data_qwxyz_1['__time']
    data_euler_1['roll'], data_euler_1['pitch'], data_euler_1['yaw'] = quat2euler(
        data_qwxyz_1['/beetle1/uav/cog/odom/pose/pose/orientation/w'], data_qwxyz_1['/beetle1/uav/cog/odom/pose/pose/orientation/x'],
        data_qwxyz_1['/beetle1/uav/cog/odom/pose/pose/orientation/y'], data_qwxyz_1['/beetle1/uav/cog/odom/pose/pose/orientation/z'])

    data_euler_2 = pd.DataFrame()
    data_euler_2['__time'] = data_qwxyz_2['__time']
    data_euler_2['roll'], data_euler_2['pitch'], data_euler_2['yaw'] = quat2euler(
        data_qwxyz_2['/beetle1/uav/cog/odom/pose/pose/orientation/w'], data_qwxyz_2['/beetle1/uav/cog/odom/pose/pose/orientation/x'],
        data_qwxyz_2['/beetle1/uav/cog/odom/pose/pose/orientation/y'], data_qwxyz_2['/beetle1/uav/cog/odom/pose/pose/orientation/z'])

    # thrust_cmd
    data_thrust_cmd = data_1[
        ['__time', '/beetle1/four_axes/command/base_thrust[0]', '/beetle1/four_axes/command/base_thrust[1]',
         '/beetle1/four_axes/command/base_thrust[2]', '/beetle1/four_axes/command/base_thrust[3]']]
    data_thrust_cmd = data_thrust_cmd.dropna()

    # servo angle cmd
    data_servo_angle_cmd = data_1[
        ['__time', '/beetle1/gimbals_ctrl/gimbal1/position', '/beetle1/gimbals_ctrl/gimbal2/position',
         '/beetle1/gimbals_ctrl/gimbal3/position', '/beetle1/gimbals_ctrl/gimbal4/position']]
    data_servo_angle_cmd = data_servo_angle_cmd.dropna()

    # ======= plotting =========
    plt.style.use(["science", "grid"])

    plt.rcParams.update({'font.size': 11})  # default is 10
    label_size = 14

    fig = plt.figure(figsize=(7, 7))

    t_bias_1 = max(data_xyz_1['__time'].iloc[0], data_xyz_ref_1['__time'].iloc[0])
    t_bias_2 = max(data_xyz_2['__time'].iloc[0], data_xyz_ref_2['__time'].iloc[0])
    color_ref = '#0C5DA5'
    color_real = '#FF2C00'

    # --------------------------------
    plt.subplot(4, 2, 1)
    t_ref_1 = np.array(data_xyz_ref_1['__time']) - t_bias_1
    x_ref_1 = np.array(data_xyz_ref_1['/beetle1/set_ref_traj/x/data[0]'])
    plt.plot(t_ref_1, x_ref_1, label='ref-2x', linestyle="--")

    t_ref_2 = np.array(data_xyz_ref_2['__time']) - t_bias_2
    x_ref_2 = np.array(data_xyz_ref_2['/beetle1/set_ref_traj/x/data[0]'])
    plt.plot(t_ref_2, x_ref_2, label='ref-1x', linestyle="--")

    t_1 = np.array(data_xyz_1['__time']) - t_bias_1
    x_1 = np.array(data_xyz_1['/beetle1/uav/cog/odom/pose/pose/position/x'])
    plt.plot(t_1, x_1, label='real-2x')

    t_2 = np.array(data_xyz_2['__time']) - t_bias_2
    x_2 = np.array(data_xyz_2['/beetle1/uav/cog/odom/pose/pose/position/x'])
    plt.plot(t_2, x_2, label='real-1x')

    plt.legend(framealpha=legend_alpha, loc='upper right')
    plt.ylabel('X (m)', fontsize=label_size)

    # calculate RMSE
    rmse_x_1 = calculate_rmse(t_1, x_1, t_ref_1, x_ref_1)
    print(f'RMSE X 1 (m): {rmse_x_1}')

    rmse_x_2 = calculate_rmse(t_2, x_2, t_ref_2, x_ref_2)
    print(f'RMSE X 2 (m): {rmse_x_2}')

    # --------------------------------
    plt.subplot(4, 2, 2)
    t_ref_1 = np.array(data_euler_ref_1['__time']) - t_bias_1
    roll_ref_1 = np.array(data_euler_ref_1['roll'])
    plt.plot(t_ref_1, roll_ref_1 * 180 / np.pi, label='ref', linestyle="--")

    t_ref_2 = np.array(data_euler_ref_2['__time']) - t_bias_2
    roll_ref_2 = np.array(data_euler_ref_2['roll'])
    plt.plot(t_ref_2, roll_ref_2 * 180 / np.pi, label='ref', linestyle="--")

    t_1 = np.array(data_euler_1['__time']) - t_bias_1
    roll_1 = np.array(data_euler_1['roll'])
    plt.plot(t_1, roll_1 * 180 / np.pi, label='real')

    t_2 = np.array(data_euler_2['__time']) - t_bias_2
    roll_2 = np.array(data_euler_2['roll'])
    plt.plot(t_2, roll_2 * 180 / np.pi, label='real')

    plt.ylabel('Roll ($^\\circ$)', fontsize=label_size)

    # calculate RMSE
    rmse_roll_1 = calculate_rmse(t_1, roll_1, t_ref_1, roll_ref_1)
    print(f'RMSE Roll 1 (rad): {rmse_roll_1}')
    print(f'RMSE Roll 1 (deg): {rmse_roll_1 * 180 / np.pi}')

    rmse_roll_2 = calculate_rmse(t_2, roll_2, t_ref_2, roll_ref_2)
    print(f'RMSE Roll 2 (rad): {rmse_roll_2}')
    print(f'RMSE Roll 2 (deg): {rmse_roll_2 * 180 / np.pi}')

    # --------------------------------
    plt.subplot(4, 2, 3)
    t_ref_1 = np.array(data_xyz_ref_1['__time']) - t_bias_1
    y_ref_1 = np.array(data_xyz_ref_1['/beetle1/set_ref_traj/x/data[1]'])
    plt.plot(t_ref_1, y_ref_1, label='ref', linestyle="--")

    t_ref_2 = np.array(data_xyz_ref_2['__time']) - t_bias_2
    y_ref_2 = np.array(data_xyz_ref_2['/beetle1/set_ref_traj/x/data[1]'])
    plt.plot(t_ref_2, y_ref_2, label='ref', linestyle="--")

    t_1 = np.array(data_xyz_1['__time']) - t_bias_1
    y_1 = np.array(data_xyz_1['/beetle1/uav/cog/odom/pose/pose/position/y'])
    plt.plot(t_1, y_1, label='Y')

    t_2 = np.array(data_xyz_2['__time']) - t_bias_2
    y_2 = np.array(data_xyz_2['/beetle1/uav/cog/odom/pose/pose/position/y'])
    plt.plot(t_2, y_2, label='Y')

    plt.ylabel('Y (m)', fontsize=label_size)

    # calculate RMSE
    rmse_y_1 = calculate_rmse(t_1, y_1, t_ref_1, y_ref_1)
    print(f'RMSE Y 1 (m): {rmse_y_1}')

    rmse_y_2 = calculate_rmse(t_2, y_2, t_ref_2, y_ref_2)
    print(f'RMSE Y 2 (m): {rmse_y_2}')

    # --------------------------------
    plt.subplot(4, 2, 4)
    t_ref_1 = np.array(data_euler_ref_1['__time']) - t_bias_1
    pitch_ref_1 = np.array(data_euler_ref_1['pitch'])
    plt.plot(t_ref_1, pitch_ref_1 * 180 / np.pi, label='ref', linestyle="--")

    t_ref_2 = np.array(data_euler_ref_2['__time']) - t_bias_2
    pitch_ref_2 = np.array(data_euler_ref_2['pitch'])
    plt.plot(t_ref_2, pitch_ref_2 * 180 / np.pi, label='ref', linestyle="--")

    t_1 = np.array(data_euler_1['__time']) - t_bias_1
    pitch_1 = np.array(data_euler_1['pitch'])
    plt.plot(t_1, pitch_1 * 180 / np.pi, label='real')

    t_2 = np.array(data_euler_2['__time']) - t_bias_2
    pitch_2 = np.array(data_euler_2['pitch'])
    plt.plot(t_2, pitch_2 * 180 / np.pi, label='real')

    plt.ylabel('Pitch ($^\\circ$)', fontsize=label_size)

    # calculate RMSE
    rmse_pitch_1 = calculate_rmse(t_1, pitch_1, t_ref_1, pitch_ref_1)
    print(f'RMSE Pitch 1 (rad): {rmse_pitch_1}')
    print(f'RMSE Pitch 1 (deg): {rmse_pitch_1 * 180 / np.pi}')

    rmse_pitch_2 = calculate_rmse(t_2, pitch_2, t_ref_2, pitch_ref_2)
    print(f'RMSE Pitch 2 (rad): {rmse_pitch_2}')
    print(f'RMSE Pitch 2 (deg): {rmse_pitch_2 * 180 / np.pi}')

    # --------------------------------
    plt.subplot(4, 2, 5)
    t_ref_1 = np.array(data_xyz_ref_1['__time']) - t_bias_1
    z_ref_1 = np.array(data_xyz_ref_1['/beetle1/set_ref_traj/x/data[2]'])
    plt.plot(t_ref_1, z_ref_1, label='ref', linestyle="--")

    t_ref_2 = np.array(data_xyz_ref_2['__time']) - t_bias_2
    z_ref_2 = np.array(data_xyz_ref_2['/beetle1/set_ref_traj/x/data[2]'])
    plt.plot(t_ref_2, z_ref_2, label='ref', linestyle="--")

    t_1 = np.array(data_xyz_1['__time']) - t_bias_1
    z_1 = np.array(data_xyz_1['/beetle1/uav/cog/odom/pose/pose/position/z'])
    plt.plot(t_1, z_1, label='Z')
    plt.ylabel('Z (m)', fontsize=label_size)

    t_2 = np.array(data_xyz_2['__time']) - t_bias_2
    z_2 = np.array(data_xyz_2['/beetle1/uav/cog/odom/pose/pose/position/z'])
    plt.plot(t_2, z_2, label='Z')

    # calculate RMSE
    rmse_z_1 = calculate_rmse(t_1, z_1, t_ref_1, z_ref_1)
    print(f'RMSE Z 1 (m): {rmse_z_1}')

    rmse_z_2 = calculate_rmse(t_2, z_2, t_ref_2, z_ref_2)
    print(f'RMSE Z 2 (m): {rmse_z_2}')

    # --------------------------------
    plt.subplot(4, 2, 6)
    t_ref_1 = np.array(data_euler_ref_1['__time']) - t_bias_1
    yaw_ref_1 = np.array(data_euler_ref_1['yaw'])
    # if yaw_ref has a jump, we need to fix it
    for i in range(1, len(yaw_ref_1)):
        if yaw_ref_1[i] - yaw_ref_1[i - 1] > np.pi:
            yaw_ref_1[i:] -= 2 * np.pi
        elif yaw_ref_1[i] - yaw_ref_1[i - 1] < -np.pi:
            yaw_ref_1[i:] += 2 * np.pi
    plt.plot(t_ref_1, yaw_ref_1 * 180 / np.pi, label='ref', linestyle="--")

    t_ref_2 = np.array(data_euler_ref_2['__time']) - t_bias_2
    yaw_ref_2 = np.array(data_euler_ref_2['yaw'])
    # if yaw_ref has a jump, we need to fix it
    for i in range(1, len(yaw_ref_2)):
        if yaw_ref_2[i] - yaw_ref_2[i - 1] > np.pi:
            yaw_ref_2[i:] -= 2 * np.pi
        elif yaw_ref_2[i] - yaw_ref_2[i - 1] < -np.pi:
            yaw_ref_2[i:] += 2 * np.pi
    plt.plot(t_ref_2, yaw_ref_2 * 180 / np.pi, label='ref', linestyle="--")

    t_1 = np.array(data_euler_1['__time']) - t_bias_1
    yaw_1 = np.array(data_euler_1['yaw'])
    # if yaw has a jump, we need to fix it
    for i in range(1, len(yaw_1)):
        if yaw_1[i] - yaw_1[i - 1] > np.pi:
            yaw_1[i:] -= 2 * np.pi
        elif yaw_1[i] - yaw_1[i - 1] < -np.pi:
            yaw_1[i:] += 2 * np.pi
    plt.plot(t_1, yaw_1 * 180 / np.pi, label='real')

    t_2 = np.array(data_euler_2['__time']) - t_bias_2
    yaw_2 = np.array(data_euler_2['yaw'])
    # if yaw has a jump, we need to fix it
    for i in range(1, len(yaw_2)):
        if yaw_2[i] - yaw_2[i - 1] > np.pi:
            yaw_2[i:] -= 2 * np.pi
        elif yaw_2[i] - yaw_2[i - 1] < -np.pi:
            yaw_2[i:] += 2 * np.pi
    plt.plot(t_2, yaw_2 * 180 / np.pi, label='real')

    plt.ylabel('Yaw ($^\\circ$)', fontsize=label_size)

    # calculate RMSE
    rmse_yaw_1 = calculate_rmse(t_1, yaw_1, t_ref_1, yaw_ref_1, is_yaw=True)
    print(f'RMSE Yaw 1 (rad): {rmse_yaw_1}')
    print(f'RMSE Yaw 1 (deg): {rmse_yaw_1 * 180 / np.pi}')

    rmse_yaw_2 = calculate_rmse(t_2, yaw_2, t_ref_2, yaw_ref_2, is_yaw=True)
    print(f'RMSE Yaw 2 (rad): {rmse_yaw_2}')
    print(f'RMSE Yaw 2 (deg): {rmse_yaw_2 * 180 / np.pi}')

    # --------------------------------
    plt.subplot(4, 2, 7)
    t_1 = np.array(data_thrust_cmd['__time']) - t_bias_1
    thrust1 = np.array(data_thrust_cmd['/beetle1/four_axes/command/base_thrust[0]'])
    plt.plot(t_1, thrust1, label='$f_{c1}$')
    thrust2 = np.array(data_thrust_cmd['/beetle1/four_axes/command/base_thrust[1]'])
    plt.plot(t_1, thrust2, label='$f_{c2}$')
    thrust3 = np.array(data_thrust_cmd['/beetle1/four_axes/command/base_thrust[2]'])
    plt.plot(t_1, thrust3, label='$f_{c3}$')
    thrust4 = np.array(data_thrust_cmd['/beetle1/four_axes/command/base_thrust[3]'])
    plt.plot(t_1, thrust4, label='$f_{c4}$')
    plt.ylabel('Thrust Cmd (N)', fontsize=label_size)
    plt.xlabel('Time (s)', fontsize=label_size)
    plt.legend(framealpha=legend_alpha, loc='upper left')

    # --------------------------------
    plt.subplot(4, 2, 8)
    t_1 = np.array(data_servo_angle_cmd['__time']) - t_bias_1
    servo1 = np.array(data_servo_angle_cmd['/beetle1/gimbals_ctrl/gimbal1/position']) * 180 / np.pi
    plt.plot(t_1, servo1, label='$\\alpha_{c1}$')
    servo2 = np.array(data_servo_angle_cmd['/beetle1/gimbals_ctrl/gimbal2/position']) * 180 / np.pi
    plt.plot(t_1, servo2, label='$\\alpha_{c2}$')
    servo3 = np.array(data_servo_angle_cmd['/beetle1/gimbals_ctrl/gimbal3/position']) * 180 / np.pi
    plt.plot(t_1, servo3, label='$\\alpha_{c3}$')
    servo4 = np.array(data_servo_angle_cmd['/beetle1/gimbals_ctrl/gimbal4/position']) * 180 / np.pi
    plt.plot(t_1, servo4, label='$\\alpha_{c4}$')
    plt.ylabel('Servo Cmd ($^\\circ$)', fontsize=label_size)
    plt.xlabel('Time (s)', fontsize=label_size)
    plt.legend(framealpha=legend_alpha, loc='upper left')

    # --------------------------------
    plt.tight_layout()
    # make the subplots very compact
    fig.subplots_adjust(hspace=0.2)
    plt.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Plot the trajectory. Please use plotjuggler to generate the csv file.')
    parser.add_argument('file_path_1', type=str, help='The file name of the trajectory 1')
    parser.add_argument('file_path_2', type=str, help='The file name of the trajectory 2')

    args = parser.parse_args()

    main(args.file_path_1, args.file_path_2)
