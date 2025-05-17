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
    # fly_data = pd.read_csv(fly_file_path)
    sensor_data = pd.read_csv(sensor_file_path)

    # ======= data selection =========
    # data_est_wrench = fly_data[
    #     ['__time', '/beetle1/disturbance_wrench/wrench/force/x', '/beetle1/disturbance_wrench/wrench/force/y',
    #         '/beetle1/disturbance_wrench/wrench/force/z', '/beetle1/disturbance_wrench/wrench/torque/x',
    #         '/beetle1/disturbance_wrench/wrench/torque/y', '/beetle1/disturbance_wrench/wrench/torque/z']]
    # data_est_wrench = data_est_wrench.dropna()
    # data_est_wrench.columns = ['__time', 'fx', 'fy', 'fz', 'tx', 'ty', 'tz']

    data_sen_wrench = sensor_data[
        ['__time', '/cfs/data/wrench/force/x', '/cfs/data/wrench/force/y', '/cfs/data/wrench/force/z',
         '/cfs/data/wrench/torque/x', '/cfs/data/wrench/torque/y', '/cfs/data/wrench/torque/z']]
    data_sen_wrench = data_sen_wrench.dropna()
    data_sen_wrench.columns = ['t', 'fx', 'fy', 'fz', 'tx', 'ty', 'tz']


    # # ======= orientation =========
    # data_qwxyz = fly_data[
    #     ['__time', '/beetle1/uav/cog/odom/pose/pose/orientation/w', '/beetle1/uav/cog/odom/pose/pose/orientation/x',
    #      '/beetle1/uav/cog/odom/pose/pose/orientation/y', '/beetle1/uav/cog/odom/pose/pose/orientation/z']]
    #
    # data_qwxyz = data_qwxyz.dropna()
    # data_qwxyz.columns = ['__time', 'qw', 'qx', 'qy', 'qz']

    # ======= plotting =========
    if plot_type == 0:
        plt.style.use(["science", "grid"])

        plt.rcParams.update({'font.size': 11})  # default is 10
        label_size = 14

        fig, axes = plt.subplots(3, 2, sharex=True, figsize=(7, 7))
        axes = axes.flatten()   # makes it easy to index 0–5

        color_ref = '#0C5DA5'
        color_real = '#FF2C00'

        t_ref = np.array(data_sen_wrench['t']) - data_sen_wrench['t'].iloc[0]

        # --------------------------------
        ax = axes[0]
        fx_ref = np.array(data_sen_wrench['fx'])
        ax.plot(t_ref, fx_ref, label='ref', linestyle="--", color=color_ref)

        ax.legend(framealpha=legend_alpha)
        ax.set_ylabel('${^B}f_x$ [N]', fontsize=label_size)

        # --------------------------------
        ax = axes[1]
        tx_ref = np.array(data_sen_wrench['tx'])
        ax.plot(t_ref, tx_ref, label='ref', linestyle="--", color=color_ref)

        ax.set_ylabel('${^B}\\tau_x$ [N$\\cdot$m]', fontsize=label_size)

        # --------------------------------
        ax = axes[2]
        fy_ref = np.array(data_sen_wrench['fy'])
        ax.plot(t_ref, fy_ref, label='ref', linestyle="--", color=color_ref)

        ax.set_ylabel('${^B}f_y$ [N]', fontsize=label_size)

        # --------------------------------
        ax = axes[3]
        ty_ref = np.array(data_sen_wrench['ty'])
        ax.plot(t_ref, ty_ref, label='ref', linestyle="--", color=color_ref)

        ax.set_ylabel('${^B}\\tau_y$ [N$\\cdot$m]', fontsize=label_size)

        # --------------------------------
        ax = axes[4]
        fz_ref = np.array(data_sen_wrench['fz'])
        ax.plot(t_ref, fz_ref, label='ref', linestyle="--", color=color_ref)

        ax.set_ylabel('${^B}f_z$ [N]', fontsize=label_size)
        ax.set_xlabel('Time [s]', fontsize=label_size)

        # --------------------------------
        ax = axes[5]
        tz_ref = np.array(data_sen_wrench['tz'])
        ax.plot(t_ref, tz_ref, label='ref', linestyle="--", color=color_ref)

        ax.set_ylabel('${^B}\\tau_z$ [N$\\cdot$m]', fontsize=label_size)
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
