import pandas as pd
import numpy as np
import scienceplots
import matplotlib.pyplot as plt
import matplotlib.patches as patches
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


def main(file_path, plot_type):
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

    color_ref = '#0072BD'
    color_real = '#D95319'

    t_bias = max(data_xyz['__time'].iloc[0], data_xyz_ref['__time'].iloc[0])

    if plot_type == 0:
        fig = plt.figure(figsize=(5, 5))

    if plot_type == 1:
        fig = plt.figure(figsize=(5, 3))

        # ----------
        # Plot the x and y position
        data_xyz_ref['__time'] = pd.to_datetime(data_xyz_ref['__time'])
        data_xyz_arm['__time'] = pd.to_datetime(data_xyz_arm['__time'])
        data_xyz_ref = data_xyz_ref.set_index('__time')
        data_xyz_arm = data_xyz_arm.set_index('__time')

        # Remove duplicate time tags and only keep the first occurrence
        data_xyz_ref = data_xyz_ref[~data_xyz_ref.index.duplicated(keep='first')]
        data_xyz_arm = data_xyz_arm[~data_xyz_arm.index.duplicated(keep='first')]

        # Construct a common time index
        common_time = data_xyz_ref.index.union(data_xyz_arm.index).sort_values()

        # Reindex and align data using time interpolation
        data_xyz_ref_aligned = data_xyz_ref.reindex(common_time).interpolate(method='time')
        data_xyz_arm_aligned = data_xyz_arm.reindex(common_time).interpolate(method='time')

        # Reset the index if necessary
        data_xyz_ref_aligned = data_xyz_ref_aligned.reset_index()
        data_xyz_arm_aligned = data_xyz_arm_aligned.reset_index()

        x = np.array(data_xyz_ref_aligned['/hand/mocap/pose/pose/position/x'] - data_xyz_arm_aligned[
            '/arm/mocap/pose/pose/position/x'])
        y = np.array(data_xyz_ref_aligned['/hand/mocap/pose/pose/position/y'] - data_xyz_arm_aligned[
            '/arm/mocap/pose/pose/position/y'])

        # x = np.array(data_xyz_ref['/hand/mocap/pose/pose/position/x'])
        # y = np.array(data_xyz_ref['/hand/mocap/pose/pose/position/y'])

        plt.plot(x, y, label='robot', linestyle="--", color=color_ref)

        plt.xlabel('X [m]', fontsize=label_size)
        plt.ylabel('Y [m]', fontsize=label_size)

        plt.xlim(0.0, 0.40)
        plt.ylim(0.0, 0.35)

        # set 1:1
        plt.gca().set_aspect('equal', adjustable='box')

        # plot the first point as a star
        plt.plot(x[0], y[0], 'r*', markersize=10)

        # plot the position of shoulder
        plt.plot(0.0, 0.0, 'rP', markersize=10)

        # plot a green circle for stop zone
        circle = patches.Circle((0.0, 0.0), 0.2, edgecolor='green', facecolor='none', alpha=0.2, linewidth=2)
        plt.gca().add_patch(circle)

        circle = patches.Circle((0.0, 0.0), 0.4, edgecolor='green', facecolor='none', alpha=0.2, linewidth=2)
        plt.gca().add_patch(circle)

    if plot_type == 2:
        fig = plt.figure(figsize=(5, 3))

        # ----------
        # Plot the x and y position
        data_xyz_ref['__time'] = pd.to_datetime(data_xyz_ref['__time'])
        data_xyz_arm['__time'] = pd.to_datetime(data_xyz_arm['__time'])
        data_xyz_ref = data_xyz_ref.set_index('__time')
        data_xyz_arm = data_xyz_arm.set_index('__time')

        # Remove duplicate time tags and only keep the first occurrence
        data_xyz_ref = data_xyz_ref[~data_xyz_ref.index.duplicated(keep='first')]
        data_xyz_arm = data_xyz_arm[~data_xyz_arm.index.duplicated(keep='first')]

        # Construct a common time index
        common_time = data_xyz_ref.index.union(data_xyz_arm.index).sort_values()

        # Reindex and align data using time interpolation
        data_xyz_ref_aligned = data_xyz_ref.reindex(common_time).interpolate(method='time')
        data_xyz_arm_aligned = data_xyz_arm.reindex(common_time).interpolate(method='time')

        # Reset the index if necessary
        data_xyz_ref_aligned = data_xyz_ref_aligned.reset_index()
        data_xyz_arm_aligned = data_xyz_arm_aligned.reset_index()

        x = np.array(data_xyz_ref_aligned['/hand/mocap/pose/pose/position/x'] - data_xyz_arm_aligned[
            '/arm/mocap/pose/pose/position/x'])
        y = np.array(data_xyz_ref_aligned['/hand/mocap/pose/pose/position/y'] - data_xyz_arm_aligned[
            '/arm/mocap/pose/pose/position/y'])

        # x = np.array(data_xyz_ref['/hand/mocap/pose/pose/position/x'])
        # y = np.array(data_xyz_ref['/hand/mocap/pose/pose/position/y'])

        plt.plot(x, y, label='robot', linestyle="--", color=color_ref)

        plt.xlabel('X [m]', fontsize=label_size)
        plt.ylabel('Y [m]', fontsize=label_size)

        plt.xlim(0.0, 0.5)
        plt.ylim(-0.3, 0.3)

        # set 1:1
        plt.gca().set_aspect('equal', adjustable='box')

        # plot the first point as a star
        plt.plot(x[0], y[0], 'r*', markersize=10)

        # plot the position of shoulder
        plt.plot(0.0, 0.0, 'rP', markersize=10)

        # plot a green circle for stop zone
        circle = patches.Circle((0.3, 0.0), 0.15, edgecolor='none', facecolor='green', alpha=0.2, linewidth=2)
        plt.gca().add_patch(circle)

        # plot the position of the origin of J coordinate
        plt.plot(0.3, 0.0, 'ro', markersize=5)

    # --- Common settings ---
    plt.tight_layout()
    fig.subplots_adjust(hspace=0.2)
    plt.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Plot the trajectory. Please use plotjuggler to generate the csv file.')
    parser.add_argument('file_path', type=str, help='The file name of the trajectory')
    parser.add_argument('--type', type=int, default=0, help='The type of plotting: 0: X-Y; 1: Spherical; 2: Cartesian')

    args = parser.parse_args()

    main(args.file_path, args.type)
