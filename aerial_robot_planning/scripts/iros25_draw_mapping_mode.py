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


def main(file_path):
    # Load the data from csv file
    data = pd.read_csv(file_path)

    # ======= xyz =========
    data_xyz = data[
        ['__time', '/beetle1/uav/cog/odom/pose/pose/position/x', '/beetle1/uav/cog/odom/pose/pose/position/y',
         '/beetle1/uav/cog/odom/pose/pose/position/z']]

    data_xyz_ref = data[
        ['__time', '/hand/mocap/pose/pose/position/x', '/hand/mocap/pose/pose/position/y',
         '/hand/mocap/pose/pose/position/z']]

    data_xyz = data_xyz.dropna()
    data_xyz_ref = data_xyz_ref.dropna()

    # ======= rpy =========
    data_qwxyz = data[
        ['__time', '/beetle1/uav/cog/odom/pose/pose/orientation/w', '/beetle1/uav/cog/odom/pose/pose/orientation/x',
         '/beetle1/uav/cog/odom/pose/pose/orientation/y', '/beetle1/uav/cog/odom/pose/pose/orientation/z']]

    data_qwxyz_ref = data[
        ['__time', '/hand/mocap/pose/pose/orientation/w', '/hand/mocap/pose/pose/orientation/x',
         '/hand/mocap/pose/pose/orientation/y', '/hand/mocap/pose/pose/orientation/z']]

    data_qwxyz_ref = data_qwxyz_ref.dropna()
    data_qwxyz = data_qwxyz.dropna()

    # convert to euler
    data_euler_ref = pd.DataFrame()
    data_euler_ref['__time'] = data_qwxyz_ref['__time']
    data_euler_ref = pd.DataFrame()
    data_euler_ref['__time'] = data_qwxyz_ref['__time']
    data_euler_ref['roll'], data_euler_ref['pitch'], data_euler_ref['yaw'] = quat2euler(
        data_qwxyz_ref['/hand/mocap/pose/pose/orientation/w'], data_qwxyz_ref['/hand/mocap/pose/pose/orientation/x'],
        data_qwxyz_ref['/hand/mocap/pose/pose/orientation/y'], data_qwxyz_ref['/hand/mocap/pose/pose/orientation/z'])

    data_euler = pd.DataFrame()
    data_euler['__time'] = data_qwxyz['__time']
    data_euler['roll'], data_euler['pitch'], data_euler['yaw'] = quat2euler(
        data_qwxyz['/beetle1/uav/cog/odom/pose/pose/orientation/w'],
        data_qwxyz['/beetle1/uav/cog/odom/pose/pose/orientation/x'],
        data_qwxyz['/beetle1/uav/cog/odom/pose/pose/orientation/y'],
        data_qwxyz['/beetle1/uav/cog/odom/pose/pose/orientation/z'])

    # ======= Plotting =========
    plt.style.use(["science", "grid"])
    plt.rcParams.update({'font.size': 11})  # default is 10
    label_size = 14
    legend_alpha = 0.5  # Set transparency for the legend if needed

    # Using 7 subplots: first 3 for position, last 4 for quaternion
    fig = plt.figure(figsize=(7, 9))
    t_bias = max(data_xyz['__time'].iloc[0], data_xyz_ref['__time'].iloc[0])
    color_ref = '#0072BD'
    color_real = '#D95319'

    time_start_rotate = 28.6993
    time_stop_rotate = 35.2177

    # --- Subplot 1: X position ---
    plt.subplot(7, 1, 1)
    t_ref = np.array(data_xyz_ref['__time']) - t_bias
    x_ref = np.array(data_xyz_ref['/hand/mocap/pose/pose/position/x'])
    plt.plot(t_ref, x_ref, label='hand', linestyle="--", color=color_ref)

    t = np.array(data_xyz['__time']) - t_bias
    x = np.array(data_xyz['/beetle1/uav/cog/odom/pose/pose/position/x'])
    plt.plot(t, x, label='robot', color=color_real)
    plt.legend(framealpha=legend_alpha)
    plt.ylabel('X (m)', fontsize=label_size)
    plt.xlim(0, 50)
    plt.axvspan(time_start_rotate, time_stop_rotate, facecolor="#EDB120", alpha=0.3)

    rmse_x = calculate_rmse(t, x, t_ref, x_ref)
    print(f'RMSE X (m): {rmse_x}')

    # --- Subplot 2: Y position ---
    plt.subplot(7, 1, 2)
    t_ref = np.array(data_xyz_ref['__time']) - t_bias
    y_ref = np.array(data_xyz_ref['/hand/mocap/pose/pose/position/y'])
    plt.plot(t_ref, y_ref, label='hand', linestyle="--", color=color_ref)

    t = np.array(data_xyz['__time']) - t_bias
    y = np.array(data_xyz['/beetle1/uav/cog/odom/pose/pose/position/y'])
    plt.plot(t, y, label='robot', color=color_real)
    # plt.legend(framealpha=legend_alpha)
    plt.ylabel('Y (m)', fontsize=label_size)
    plt.xlim(0, 50)
    plt.axvspan(time_start_rotate, time_stop_rotate, facecolor="#EDB120", alpha=0.3)

    rmse_y = calculate_rmse(t, y, t_ref, y_ref)
    print(f'RMSE Y (m): {rmse_y}')

    # --- Subplot 3: Z position ---
    plt.subplot(7, 1, 3)
    t_ref = np.array(data_xyz_ref['__time']) - t_bias
    z_ref = np.array(data_xyz_ref['/hand/mocap/pose/pose/position/z'])
    plt.plot(t_ref, z_ref, label='hand', linestyle="--", color=color_ref)

    t = np.array(data_xyz['__time']) - t_bias
    z = np.array(data_xyz['/beetle1/uav/cog/odom/pose/pose/position/z'])
    plt.plot(t, z, label='robot', color=color_real)
    # plt.legend(framealpha=legend_alpha)
    plt.ylabel('Z (m)', fontsize=label_size)
    plt.xlim(0, 50)
    plt.axvspan(time_start_rotate, time_stop_rotate, facecolor="#EDB120", alpha=0.3)

    rmse_z = calculate_rmse(t, z, t_ref, z_ref)
    print(f'RMSE Z (m): {rmse_z}')

    # --- Subplot 4: Quaternion w ---
    plt.subplot(7, 1, 4)
    t_ref = np.array(data_qwxyz_ref['__time']) - t_bias
    qw_ref = -np.array(data_qwxyz_ref['/hand/mocap/pose/pose/orientation/w'])
    plt.plot(t_ref, qw_ref, label='hand', linestyle="--", color=color_ref)

    t = np.array(data_qwxyz['__time']) - t_bias
    qw = np.array(data_qwxyz['/beetle1/uav/cog/odom/pose/pose/orientation/w'])
    plt.plot(t, qw, label='robot', color=color_real)
    plt.legend(framealpha=legend_alpha, loc='lower right')
    plt.ylabel('qw', fontsize=label_size)
    plt.xlim(0, 50)
    plt.axvspan(time_start_rotate, time_stop_rotate, facecolor="#EDB120", alpha=0.3)

    rmse_qw = calculate_rmse(t, qw, t_ref, qw_ref)
    print(f'RMSE qw: {rmse_qw}')

    # --- Subplot 5: Quaternion x ---
    plt.subplot(7, 1, 5)
    t_ref = np.array(data_qwxyz_ref['__time']) - t_bias
    qx_ref = -np.array(data_qwxyz_ref['/hand/mocap/pose/pose/orientation/x'])
    plt.plot(t_ref, qx_ref, label='hand', linestyle="--", color=color_ref)

    t = np.array(data_qwxyz['__time']) - t_bias
    qx = np.array(data_qwxyz['/beetle1/uav/cog/odom/pose/pose/orientation/x'])
    plt.plot(t, qx, label='robot', color=color_real)
    # plt.legend(framealpha=legend_alpha)
    plt.ylabel('qx', fontsize=label_size)
    plt.xlim(0, 50)
    plt.axvspan(time_start_rotate, time_stop_rotate, facecolor="#EDB120", alpha=0.3)

    rmse_qx = calculate_rmse(t, qx, t_ref, qx_ref)
    print(f'RMSE qx: {rmse_qx}')

    # --- Subplot 6: Quaternion y ---
    plt.subplot(7, 1, 6)
    t_ref = np.array(data_qwxyz_ref['__time']) - t_bias
    qy_ref = -np.array(data_qwxyz_ref['/hand/mocap/pose/pose/orientation/y'])
    plt.plot(t_ref, qy_ref, label='hand', linestyle="--", color=color_ref)

    t = np.array(data_qwxyz['__time']) - t_bias
    qy = np.array(data_qwxyz['/beetle1/uav/cog/odom/pose/pose/orientation/y'])
    plt.plot(t, qy, label='robot', color=color_real)
    # plt.legend(framealpha=legend_alpha)
    plt.ylabel('qy', fontsize=label_size)
    plt.xlim(0, 50)
    plt.axvspan(time_start_rotate, time_stop_rotate, facecolor="#EDB120", alpha=0.3)

    rmse_qy = calculate_rmse(t, qy, t_ref, qy_ref)
    print(f'RMSE qy: {rmse_qy}')

    # --- Subplot 7: Quaternion z ---
    plt.subplot(7, 1, 7)
    t_ref = np.array(data_qwxyz_ref['__time']) - t_bias
    qz_ref = -np.array(data_qwxyz_ref['/hand/mocap/pose/pose/orientation/z'])
    plt.plot(t_ref, qz_ref, label='hand', linestyle="--", color=color_ref)

    t = np.array(data_qwxyz['__time']) - t_bias
    qz = np.array(data_qwxyz['/beetle1/uav/cog/odom/pose/pose/orientation/z'])
    plt.plot(t, qz, label='robot', color=color_real)
    # plt.legend(framealpha=legend_alpha)
    plt.ylabel('qz', fontsize=label_size)
    plt.xlim(0, 50)
    plt.axvspan(time_start_rotate, time_stop_rotate, facecolor="#EDB120", alpha=0.3)

    rmse_qz = calculate_rmse(t, qz, t_ref, qz_ref)
    print(f'RMSE qz: {rmse_qz}')

    # --- Common settings ---
    plt.xlabel('Time (s)', fontsize=label_size)

    plt.tight_layout()
    fig.subplots_adjust(hspace=0.2)
    plt.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Plot the trajectory. Please use plotjuggler to generate the csv file.')
    parser.add_argument('file_path', type=str, help='The file name of the trajectory')

    args = parser.parse_args()

    main(args.file_path)
