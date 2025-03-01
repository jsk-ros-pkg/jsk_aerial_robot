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

    data_xyz_arm = data[
        ['__time', '/arm/mocap/pose/pose/position/x', '/arm/mocap/pose/pose/position/y',
            '/arm/mocap/pose/pose/position/z']]

    data_xyz = data_xyz.dropna()
    data_xyz_ref = data_xyz_ref.dropna()
    data_xyz_arm = data_xyz_arm.dropna()

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

    fig = plt.figure(figsize=(5, 3))
    t_bias = max(data_xyz['__time'].iloc[0], data_xyz_ref['__time'].iloc[0])
    color_ref = '#0072BD'
    color_real = '#D95319'

    # ----------
    # Plot the x and y position

    # 假设 '__time' 列已经转换为 datetime 类型，并设置为索引
    # 如果还未转换，请先转换：
    data_xyz_ref['__time'] = pd.to_datetime(data_xyz_ref['__time'])
    data_xyz_arm['__time'] = pd.to_datetime(data_xyz_arm['__time'])
    data_xyz_ref = data_xyz_ref.set_index('__time')
    data_xyz_arm = data_xyz_arm.set_index('__time')

    # 移除重复的时间标签，只保留第一次出现的记录
    data_xyz_ref = data_xyz_ref[~data_xyz_ref.index.duplicated(keep='first')]
    data_xyz_arm = data_xyz_arm[~data_xyz_arm.index.duplicated(keep='first')]

    # 构造公共的时间索引
    common_time = data_xyz_ref.index.union(data_xyz_arm.index).sort_values()

    # 重新索引并使用时间插值对齐数据
    data_xyz_ref_aligned = data_xyz_ref.reindex(common_time).interpolate(method='time')
    data_xyz_arm_aligned = data_xyz_arm.reindex(common_time).interpolate(method='time')

    # 如果需要，可以重置索引
    data_xyz_ref_aligned = data_xyz_ref_aligned.reset_index()
    data_xyz_arm_aligned = data_xyz_arm_aligned.reset_index()

    x = np.array(data_xyz_ref_aligned['/hand/mocap/pose/pose/position/x'] - data_xyz_arm_aligned['/arm/mocap/pose/pose/position/x'])
    y = np.array(data_xyz_ref_aligned['/hand/mocap/pose/pose/position/y'] - data_xyz_arm_aligned['/arm/mocap/pose/pose/position/y'])

    # x = np.array(data_xyz_ref['/hand/mocap/pose/pose/position/x'])
    # y = np.array(data_xyz_ref['/hand/mocap/pose/pose/position/y'])

    plt.plot(x, y, label='robot', color=color_real)

    plt.xlabel('X (m)', fontsize=label_size)
    plt.ylabel('Y (m)', fontsize=label_size)

    plt.xlim(0.0, 0.38)
    plt.ylim(0.0, 0.35)

    # set 1:1
    plt.gca().set_aspect('equal', adjustable='box')

    # plot the first point as a star
    plt.plot(x[0], y[0], 'r*', markersize=10)

    # --- Common settings ---
    plt.tight_layout()
    fig.subplots_adjust(hspace=0.2)
    plt.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Plot the trajectory. Please use plotjuggler to generate the csv file.')
    parser.add_argument('file_path', type=str, help='The file name of the trajectory')

    args = parser.parse_args()

    main(args.file_path)
