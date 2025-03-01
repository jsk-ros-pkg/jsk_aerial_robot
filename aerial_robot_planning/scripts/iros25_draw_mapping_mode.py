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


matlab_blue = '#0072BD'
matlab_orange = '#D95319'
matlab_purple = "#7E2F8E"

def main(file_path, plot_type):
    # Load the data from csv file
    data = pd.read_csv(file_path)

    if plot_type == 0:
        # ======= xyz =========
        data_xyz = data[
            ['__time', '/beetle1/uav/cog/odom/pose/pose/position/x', '/beetle1/uav/cog/odom/pose/pose/position/y',
             '/beetle1/uav/cog/odom/pose/pose/position/z']].dropna()

        data_xyz_ref = data[
            ['__time', '/hand/mocap/pose/pose/position/x', '/hand/mocap/pose/pose/position/y',
             '/hand/mocap/pose/pose/position/z']].dropna()

        # ======= rpy =========
        data_qwxyz = data[
            ['__time', '/beetle1/uav/cog/odom/pose/pose/orientation/w', '/beetle1/uav/cog/odom/pose/pose/orientation/x',
             '/beetle1/uav/cog/odom/pose/pose/orientation/y', '/beetle1/uav/cog/odom/pose/pose/orientation/z']].dropna()

        data_qwxyz_ref = data[
            ['__time', '/hand/mocap/pose/pose/orientation/w', '/hand/mocap/pose/pose/orientation/x',
             '/hand/mocap/pose/pose/orientation/y', '/hand/mocap/pose/pose/orientation/z']].dropna()

        # convert to euler
        data_euler_ref = pd.DataFrame()
        data_euler_ref['__time'] = data_qwxyz_ref['__time']
        data_euler_ref = pd.DataFrame()
        data_euler_ref['__time'] = data_qwxyz_ref['__time']
        data_euler_ref['roll'], data_euler_ref['pitch'], data_euler_ref['yaw'] = quat2euler(
            data_qwxyz_ref['/hand/mocap/pose/pose/orientation/w'],
            data_qwxyz_ref['/hand/mocap/pose/pose/orientation/x'],
            data_qwxyz_ref['/hand/mocap/pose/pose/orientation/y'],
            data_qwxyz_ref['/hand/mocap/pose/pose/orientation/z'])

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

        time_start_rotate = 28.6993
        time_stop_rotate = 35.2177

        # --- Subplot 1: X position ---
        plt.subplot(7, 1, 1)
        t_ref = np.array(data_xyz_ref['__time']) - t_bias
        x_ref = np.array(data_xyz_ref['/hand/mocap/pose/pose/position/x'])

        t = np.array(data_xyz['__time']) - t_bias
        x = np.array(data_xyz['/beetle1/uav/cog/odom/pose/pose/position/x'])

        x_ref_offset = x_ref - x_ref[0] + x[0]

        plt.plot(t_ref, x_ref, label='hand', linestyle="--", color=matlab_blue)
        plt.plot(t_ref, x_ref_offset, label='hand offset', linestyle="-.", color="k")
        plt.plot(t, x, label='robot', color=matlab_orange)

        plt.legend(framealpha=legend_alpha, loc="lower right")
        plt.ylabel('X (m)', fontsize=label_size)
        plt.xlim(0, 50)
        plt.axvspan(time_start_rotate, time_stop_rotate, facecolor="#EDB120", alpha=0.3)

        rmse_x = calculate_rmse(t, x, t_ref, x_ref)
        print(f'RMSE X (m): {rmse_x}')

        # --- Subplot 2: Y position ---
        plt.subplot(7, 1, 2)
        t_ref = np.array(data_xyz_ref['__time']) - t_bias
        y_ref = np.array(data_xyz_ref['/hand/mocap/pose/pose/position/y'])

        t = np.array(data_xyz['__time']) - t_bias
        y = np.array(data_xyz['/beetle1/uav/cog/odom/pose/pose/position/y'])

        y_ref_offset = y_ref - y_ref[0] + y[0]

        plt.plot(t_ref, y_ref, label='hand', linestyle="--", color=matlab_blue)
        plt.plot(t_ref, y_ref_offset, label='hand offset', linestyle="-.", color="k")
        plt.plot(t, y, label='robot', color=matlab_orange)

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
        plt.plot(t_ref, z_ref, label='hand', linestyle="--", color=matlab_blue)

        t = np.array(data_xyz['__time']) - t_bias
        z = np.array(data_xyz['/beetle1/uav/cog/odom/pose/pose/position/z'])
        plt.plot(t, z, label='robot', color=matlab_orange)
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
        plt.plot(t_ref, qw_ref, label='hand', linestyle="--", color=matlab_blue)

        t = np.array(data_qwxyz['__time']) - t_bias
        qw = np.array(data_qwxyz['/beetle1/uav/cog/odom/pose/pose/orientation/w'])
        plt.plot(t, qw, label='robot', color=matlab_orange)
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
        plt.plot(t_ref, qx_ref, label='hand', linestyle="--", color=matlab_blue)

        t = np.array(data_qwxyz['__time']) - t_bias
        qx = np.array(data_qwxyz['/beetle1/uav/cog/odom/pose/pose/orientation/x'])
        plt.plot(t, qx, label='robot', color=matlab_orange)
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
        plt.plot(t_ref, qy_ref, label='hand', linestyle="--", color=matlab_blue)

        t = np.array(data_qwxyz['__time']) - t_bias
        qy = np.array(data_qwxyz['/beetle1/uav/cog/odom/pose/pose/orientation/y'])
        plt.plot(t, qy, label='robot', color=matlab_orange)
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
        plt.plot(t_ref, qz_ref, label='hand', linestyle="--", color=matlab_blue)

        t = np.array(data_qwxyz['__time']) - t_bias
        qz = np.array(data_qwxyz['/beetle1/uav/cog/odom/pose/pose/orientation/z'])
        plt.plot(t, qz, label='robot', color=matlab_orange)
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

    if plot_type == 1:
        # ======= 提取位置数据 =======
        data_xyz = data[
            ['__time', '/beetle1/uav/cog/odom/pose/pose/position/x',
             '/beetle1/uav/cog/odom/pose/pose/position/y',
             '/beetle1/uav/cog/odom/pose/pose/position/z']
        ].dropna()

        data_xyz_ref = data[
            ['__time', '/hand/mocap/pose/pose/position/x',
             '/hand/mocap/pose/pose/position/y',
             '/hand/mocap/pose/pose/position/z']
        ].dropna()

        # ======= 提取四元数数据 =======
        data_qwxyz = data[
            ['__time', '/beetle1/uav/cog/odom/pose/pose/orientation/w',
             '/beetle1/uav/cog/odom/pose/pose/orientation/x',
             '/beetle1/uav/cog/odom/pose/pose/orientation/y',
             '/beetle1/uav/cog/odom/pose/pose/orientation/z']
        ].dropna()

        data_qwxyz_ref = data[
            ['__time', '/hand/mocap/pose/pose/orientation/w',
             '/hand/mocap/pose/pose/orientation/x',
             '/hand/mocap/pose/pose/orientation/y',
             '/hand/mocap/pose/pose/orientation/z']
        ].dropna()

        # ======= 绘图设置 =======
        plt.style.use(["science", "grid"])
        plt.rcParams.update({'font.size': 11})
        label_size = 14
        legend_alpha = 0.5

        fig = plt.figure(figsize=(7, 9))
        # 以两个数据中较晚的初始时间作为时间偏置
        t_bias = max(data_xyz['__time'].iloc[0], data_xyz_ref['__time'].iloc[0])

        # 定义旋转区间（仅用于图中标注）
        time_start_rotate = 28.6993
        time_stop_rotate = 35.2177

        # --- Subplot 1: X 绝对误差 ---
        plt.subplot(7, 1, 1)
        t_ref = np.array(data_xyz_ref['__time']) - t_bias
        x_ref = np.array(data_xyz_ref['/hand/mocap/pose/pose/position/x'])
        t = np.array(data_xyz['__time']) - t_bias
        x = np.array(data_xyz['/beetle1/uav/cog/odom/pose/pose/position/x'])
        # 插值：将参考值对齐到机器人时间
        x_ref_interp = np.interp(t, t_ref, x_ref - x_ref[0] + x[0])
        # 计算误差
        abs_error_x = np.abs(x - x_ref_interp)

        plt.plot(t, abs_error_x, label='X', color=matlab_blue)
        # plt.legend(framealpha=legend_alpha, loc="lower right")
        plt.ylabel('Abs. Err. X (m)', fontsize=label_size)
        plt.xlim(0, 50)
        plt.axvspan(time_start_rotate, time_stop_rotate, facecolor="#EDB120", alpha=0.3)

        # --- Subplot 2: Y 绝对误差 ---
        plt.subplot(7, 1, 2)
        t_ref = np.array(data_xyz_ref['__time']) - t_bias
        y_ref = np.array(data_xyz_ref['/hand/mocap/pose/pose/position/y'])
        t = np.array(data_xyz['__time']) - t_bias
        y = np.array(data_xyz['/beetle1/uav/cog/odom/pose/pose/position/y'])
        y_ref_interp = np.interp(t, t_ref, y_ref - y_ref[0] + y[0])
        abs_error_y = np.abs(y - y_ref_interp)

        plt.plot(t, abs_error_y, label='Y', color=matlab_blue)
        # plt.legend(framealpha=legend_alpha, loc="lower right")
        plt.ylabel('Y (m)', fontsize=label_size)
        plt.xlim(0, 50)
        plt.axvspan(time_start_rotate, time_stop_rotate, facecolor="#EDB120", alpha=0.3)

        # --- Subplot 3: Z 绝对误差 ---
        plt.subplot(7, 1, 3)
        t_ref = np.array(data_xyz_ref['__time']) - t_bias
        z_ref = np.array(data_xyz_ref['/hand/mocap/pose/pose/position/z'])
        t = np.array(data_xyz['__time']) - t_bias
        z = np.array(data_xyz['/beetle1/uav/cog/odom/pose/pose/position/z'])
        z_ref_interp = np.interp(t, t_ref, z_ref)
        abs_error_z = np.abs(z - z_ref_interp)

        plt.plot(t, abs_error_z, label='Z', color=matlab_blue)
        # plt.legend(framealpha=legend_alpha, loc="lower right")
        plt.ylabel('Z (m)', fontsize=label_size)
        plt.xlim(0, 50)
        plt.axvspan(time_start_rotate, time_stop_rotate, facecolor="#EDB120", alpha=0.3)

        # --- Subplot 4: Quaternion w 绝对误差 ---
        plt.subplot(7, 1, 4)
        t_ref = np.array(data_qwxyz_ref['__time']) - t_bias
        # 注意：参考数据取负值处理
        qw_ref = -np.array(data_qwxyz_ref['/hand/mocap/pose/pose/orientation/w'])
        t = np.array(data_qwxyz['__time']) - t_bias
        qw = np.array(data_qwxyz['/beetle1/uav/cog/odom/pose/pose/orientation/w'])
        qw_ref_interp = np.interp(t, t_ref, qw_ref)
        abs_error_qw = np.abs(qw - qw_ref_interp)

        plt.plot(t, abs_error_qw, label='qw', color=matlab_blue)
        # plt.legend(framealpha=legend_alpha, loc="lower right")
        plt.ylabel('qw', fontsize=label_size)
        plt.xlim(0, 50)
        plt.axvspan(time_start_rotate, time_stop_rotate, facecolor="#EDB120", alpha=0.3)

        # --- Subplot 5: Quaternion x 绝对误差 ---
        plt.subplot(7, 1, 5)
        t_ref = np.array(data_qwxyz_ref['__time']) - t_bias
        qx_ref = -np.array(data_qwxyz_ref['/hand/mocap/pose/pose/orientation/x'])
        t = np.array(data_qwxyz['__time']) - t_bias
        qx = np.array(data_qwxyz['/beetle1/uav/cog/odom/pose/pose/orientation/x'])
        qx_ref_interp = np.interp(t, t_ref, qx_ref)
        abs_error_qx = np.abs(qx - qx_ref_interp)

        plt.plot(t, abs_error_qx, label='qx', color=matlab_blue)
        # plt.legend(framealpha=legend_alpha, loc="lower right")
        plt.ylabel('qx', fontsize=label_size)
        plt.xlim(0, 50)
        plt.axvspan(time_start_rotate, time_stop_rotate, facecolor="#EDB120", alpha=0.3)

        # --- Subplot 6: Quaternion y 绝对误差 ---
        plt.subplot(7, 1, 6)
        t_ref = np.array(data_qwxyz_ref['__time']) - t_bias
        qy_ref = -np.array(data_qwxyz_ref['/hand/mocap/pose/pose/orientation/y'])
        t = np.array(data_qwxyz['__time']) - t_bias
        qy = np.array(data_qwxyz['/beetle1/uav/cog/odom/pose/pose/orientation/y'])
        qy_ref_interp = np.interp(t, t_ref, qy_ref)
        abs_error_qy = np.abs(qy - qy_ref_interp)

        plt.plot(t, abs_error_qy, label='qy', color=matlab_blue)
        # plt.legend(framealpha=legend_alpha, loc="lower right")
        plt.ylabel('qy', fontsize=label_size)
        plt.xlim(0, 50)
        plt.axvspan(time_start_rotate, time_stop_rotate, facecolor="#EDB120", alpha=0.3)

        # --- Subplot 7: Quaternion z 绝对误差 ---
        plt.subplot(7, 1, 7)
        t_ref = np.array(data_qwxyz_ref['__time']) - t_bias
        qz_ref = -np.array(data_qwxyz_ref['/hand/mocap/pose/pose/orientation/z'])
        t = np.array(data_qwxyz['__time']) - t_bias
        qz = np.array(data_qwxyz['/beetle1/uav/cog/odom/pose/pose/orientation/z'])
        qz_ref_interp = np.interp(t, t_ref, qz_ref)
        abs_error_qz = np.abs(qz - qz_ref_interp)

        plt.plot(t, abs_error_qz, label='qz', color=matlab_blue)
        # plt.legend(framealpha=legend_alpha, loc="lower right")
        plt.ylabel('qz', fontsize=label_size)
        plt.xlim(0, 50)
        plt.axvspan(time_start_rotate, time_stop_rotate, facecolor="#EDB120", alpha=0.3)

        # --- 公共设置 ---
        plt.xlabel('Time (s)', fontsize=label_size)
        plt.tight_layout()
        fig.subplots_adjust(hspace=0.2)
        plt.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Plot the trajectory. Please use plotjuggler to generate the csv file.')
    parser.add_argument('file_path', type=str, help='The file name of the trajectory')
    parser.add_argument('--type', type=int, default=0, help='The type of plotting')

    args = parser.parse_args()

    main(args.file_path, args.type)
