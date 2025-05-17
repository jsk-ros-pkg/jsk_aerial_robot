import pandas as pd
import numpy as np
import scienceplots
import matplotlib.pyplot as plt
import argparse

from matplotlib.lines import lineStyles

legend_alpha = 0.5


def unwrap_angle_sequence(angle_seq: np.ndarray) -> np.ndarray:
    angle_seq = angle_seq.copy()  # avoid modifying the input array
    for i in range(1, len(angle_seq)):
        delta = angle_seq[i] - angle_seq[i - 1]
        if delta > np.pi:
            angle_seq[i:] -= 2 * np.pi
        elif delta < -np.pi:
            angle_seq[i:] += 2 * np.pi
    return angle_seq


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


def main(fly_file_path, sensor_file_path, plot_type):
    # Load the data from csv file
    sensor_data = pd.read_csv(sensor_file_path)
    fly_data = pd.read_csv(fly_file_path)

    # ======= data selection =========
    data_sen_wrench = sensor_data[
        ['__time', '/cfs/data/wrench/force/x', '/cfs/data/wrench/force/y', '/cfs/data/wrench/force/z',
         '/cfs/data/wrench/torque/x', '/cfs/data/wrench/torque/y', '/cfs/data/wrench/torque/z']]
    data_sen_wrench = data_sen_wrench.dropna()
    data_sen_wrench.columns = ['t', 'fx', 'fy', 'fz', 'tx', 'ty', 'tz']

    data_est_wrench = fly_data[
        ['__time', '/beetle1/disturbance_wrench/wrench/force/x', '/beetle1/disturbance_wrench/wrench/force/y',
         '/beetle1/disturbance_wrench/wrench/force/z', '/beetle1/disturbance_wrench/wrench/torque/x',
         '/beetle1/disturbance_wrench/wrench/torque/y', '/beetle1/disturbance_wrench/wrench/torque/z']]
    data_est_wrench = data_est_wrench.dropna()
    data_est_wrench.columns = ['t', 'fx', 'fy', 'fz', 'tx', 'ty', 'tz']

    data_imu = fly_data[
        ['__time', '/beetle1/imu/acc_data[0]', '/beetle1/imu/acc_data[1]', '/beetle1/imu/acc_data[2]',
         '/beetle1/imu/gyro_data[0]', '/beetle1/imu/gyro_data[1]', '/beetle1/imu/gyro_data[2]']]
    data_imu = data_imu.dropna()
    data_imu.columns = ['t', 'ax', 'ay', 'az', 'wx', 'wy', 'wz']

    # # ======= orientation =========
    # data_qwxyz = fly_data[
    #     ['__time', '/beetle1/uav/cog/odom/pose/pose/orientation/w', '/beetle1/uav/cog/odom/pose/pose/orientation/x',
    #      '/beetle1/uav/cog/odom/pose/pose/orientation/y', '/beetle1/uav/cog/odom/pose/pose/orientation/z']]
    #
    # data_qwxyz = data_qwxyz.dropna()
    # data_qwxyz.columns = ['__time', 'qw', 'qx', 'qy', 'qz']

    # ======= preprocessing =========
    t_ref = np.array(data_sen_wrench['t']) - data_sen_wrench['t'].iloc[0]
    time_duration = data_sen_wrench['t'].iloc[-1] - data_sen_wrench['t'].iloc[0]

    t_real_start = 1  # s
    t_real_end = t_real_start + time_duration
    data_est_wrench_sel = data_est_wrench[(data_est_wrench['t'] >= t_real_start + data_est_wrench['t'].iloc[0]) & (
            data_est_wrench['t'] <= data_est_wrench['t'].iloc[0] + t_real_end)]
    t_real = np.array(data_est_wrench_sel['t']) - data_est_wrench_sel['t'].iloc[0]

    data_imu_sel = data_imu[(data_imu['t'] >= t_real_start + data_imu['t'].iloc[0]) & (
            data_imu['t'] <= data_imu['t'].iloc[0] + t_real_end)]
    t_imu = np.array(data_imu_sel['t']) - data_imu_sel['t'].iloc[0]

    # ======= plotting =========
    if plot_type == 0:
        plt.style.use(["science", "grid"])

        plt.rcParams.update({'font.size': 11})  # default is 10
        label_size = 14

        fig, axes = plt.subplots(4, 2, sharex=True, figsize=(7, 7))
        axes = axes.flatten()  # makes it easy to index 0–5

        color_ref = '#0C5DA5'
        color_real = '#FF2C00'

        # --------------- Wrench -----------------
        keys = ['fx', 'tx', 'fy', 'ty', 'fz', 'tz']
        ylabels = [
            r'${}^Bf_x$ [N]', r'${}^B\tau_x$ [N$\cdot$m]',
            r'${}^Bf_y$ [N]', r'${}^B\tau_y$ [N$\cdot$m]',
            r'${}^Bf_z$ [N]', r'${}^B\tau_z$ [N$\cdot$m]',
        ]

        for i, (key, ylabel) in enumerate(zip(keys, ylabels)):
            ax = axes[i]
            # plot ref and real
            ax.plot(t_ref, data_sen_wrench[key], linestyle='--', label='ref', color=color_ref)
            ax.plot(t_real, data_est_wrench_sel[key], linestyle='-', label='real', color=color_real)

            # only the first subplot gets a legend
            if i == 0:
                ax.legend(framealpha=legend_alpha)

            ax.set_ylabel(ylabel, fontsize=label_size)

            # bottom‐row plots (i = 4,5) get the shared x‐label
            if i >= 4:
                ax.set_xlabel('Time [s]', fontsize=label_size)
        # ---------------- Imu ----------------
        gravity_const = 9.798  # m/s^2 Tokyo  # TODO: need to do coordinate transform

        ax = axes[6]
        ax.plot(t_imu, data_imu_sel['ax'], linestyle='-', label='ax')
        ax.plot(t_imu, data_imu_sel['ay'], linestyle='-.', label='ay')
        ax.plot(t_imu, data_imu_sel['az'], linestyle=':', label='az')
        ax.legend(framealpha=legend_alpha, loc='upper center', ncol=3)
        ax.set_ylabel('$^Ba$ [m/s$^2$]', fontsize=label_size)
        ax.set_xlabel('Time [s]', fontsize=label_size)
        # --------------- Gyro -----------------
        ax = axes[7]
        ax.plot(t_imu, data_imu_sel['wx'], linestyle='-', label='$\omega_x$')
        ax.plot(t_imu, data_imu_sel['wy'], linestyle='-.', label='$\omega_y$')
        ax.plot(t_imu, data_imu_sel['wz'], linestyle=':', label='$\omega_z$')
        ax.legend(framealpha=legend_alpha, loc='upper center', ncol=3)
        ax.set_ylabel('$^B\omega$ [rad/s]', fontsize=label_size)
        ax.set_xlabel('Time [s]', fontsize=label_size)

        # --------------------------------
        plt.tight_layout()

        plt.show()

    else:
        print('Invalid type')


if __name__ == '__main__':
    # python draw_external_wrench.py ~/Desktop/20250123_wrench_est.csv ~/Desktop/20250123_torque_sensor.csv
    parser = argparse.ArgumentParser(
        description='Plot the trajectory. Please use plotjuggler to generate the csv file.')
    parser.add_argument('fly_file_path', type=str, help='The file name of the trajectory')
    parser.add_argument('sensor_file_path', type=str, help='The file name of the force sensor data')
    parser.add_argument('--type', type=int, default=0, help='The type of the trajectory')

    args = parser.parse_args()

    main(args.fly_file_path, args.sensor_file_path, args.type)
