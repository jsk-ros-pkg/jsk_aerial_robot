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

    # ======= Disturbance wrench estimated by acceleration-based method =========
    data_dist_acc_force = data[
        ['__time', '/beetle1/disturbance_wrench/wrench/force/x', '/beetle1/disturbance_wrench/wrench/force/y',
         '/beetle1/disturbance_wrench/wrench/force/z']]

    data_dist_acc_torque = data[
        ['__time', '/beetle1/disturbance_wrench/wrench/torque/x', '/beetle1/disturbance_wrench/wrench/torque/y',
         '/beetle1/disturbance_wrench/wrench/torque/z']]

    data_dist_acc_force = data_dist_acc_force.dropna()
    data_dist_acc_torque = data_dist_acc_torque.dropna()

    # ======= Sensor-info =========
    data_imu_acc = data[
        ['__time', '/beetle1/imu/acc_data[0]', '/beetle1/imu/acc_data[1]', '/beetle1/imu/acc_data[2]']]
    data_imu_gyro = data[
        ['__time', '/beetle1/imu/gyro_data[0]', '/beetle1/imu/gyro_data[1]', '/beetle1/imu/gyro_data[2]']]

    data_servo_meas = data[
        ['__time', '/beetle1/joint_states/gimbal1/position', '/beetle1/joint_states/gimbal2/position',
         '/beetle1/joint_states/gimbal3/position', '/beetle1/joint_states/gimbal4/position']]
    data_rpm_meas = data[
        ['__time', '/beetle1/esc_telem/esc_telemetry_1/rpm', '/beetle1/esc_telem/esc_telemetry_2/rpm',
         '/beetle1/esc_telem/esc_telemetry_3/rpm', '/beetle1/esc_telem/esc_telemetry_4/rpm']]

    data_imu_acc = data_imu_acc.dropna()
    data_imu_gyro = data_imu_gyro.dropna()
    data_servo_meas = data_servo_meas.dropna()
    data_rpm_meas = data_rpm_meas.dropna()

    data_thrust_meas = (data_rpm_meas * 0.001) ** 2 / 7.7991

    # ======= Control commands ========
    data_servo_cmd = data[
        ['__time', '/beetle1/gimbals_ctrl/gimbal1/position', '/beetle1/gimbals_ctrl/gimbal2/position',
         '/beetle1/gimbals_ctrl/gimbal3/position', '/beetle1/gimbals_ctrl/gimbal4/position']]
    data_thrust_cmd = data[
        ['__time', '/beetle1/four_axes/command/base_thrust[0]', '/beetle1/four_axes/command/base_thrust[1]',
         '/beetle1/four_axes/command/base_thrust[2]', '/beetle1/four_axes/command/base_thrust[3]']]
    data_servo_cmd = data_servo_cmd.dropna()
    data_thrust_cmd = data_thrust_cmd.dropna()

    # ======= Plotting =========
    if type == 0:
        plt.style.use(["science", "grid"])

        plt.rcParams.update({'font.size': 11})      # Default is 10
        label_size = 14

        # Set up plot
        fig = plt.figure(figsize=(21, 7))
        color_ref = '#0C5DA5'
        color_real = '#FF2C00'

        # Time adjustment
        t_bias = max(data_dist_acc_force['__time'].iloc[0], data_dist_acc_force['__time'].iloc[0])
        tf_ref = np.array(data_dist_acc_force['__time']) - t_bias
        tt_ref = np.array(data_dist_acc_torque['__time']) - t_bias

        # Plot Forces
        plt.subplot(3, 2, 1)
        fx = np.array(data_dist_acc_force['/beetle1/disturbance_wrench/wrench/force/x'])
        plt.plot(tf_ref, fx, label='acc', linestyle="-", color=color_ref)
        plt.legend(framealpha=legend_alpha)
        plt.ylabel('$f_{dx}$ (N)', fontsize=label_size)

        plt.subplot(3, 2, 3)
        fy = np.array(data_dist_acc_force['/beetle1/disturbance_wrench/wrench/force/y'])
        plt.plot(tf_ref, fy, label='acc', linestyle="-", color=color_ref)
        plt.legend(framealpha=legend_alpha)
        plt.ylabel('$f_{dy}$ (N)', fontsize=label_size)

        plt.subplot(3, 2, 5)
        fz = np.array(data_dist_acc_force['/beetle1/disturbance_wrench/wrench/force/z'])
        plt.plot(tf_ref, fz, label='acc', linestyle="-", color=color_ref)
        plt.legend(framealpha=legend_alpha)
        plt.ylabel('$f_{dz}$ (N)', fontsize=label_size)

        # Plot Torques
        plt.subplot(3, 2, 2)
        tq_x = np.array(data_dist_acc_torque['/beetle1/disturbance_wrench/wrench/torque/x'])
        plt.plot(tt_ref, tq_x, label='acc', linestyle="-", color=color_ref)
        plt.legend(framealpha=legend_alpha)
        plt.ylabel('$\\tau_{dx}$ (N.m)', fontsize=label_size)

        plt.subplot(3, 2, 4)
        tq_y = np.array(data_dist_acc_torque['/beetle1/disturbance_wrench/wrench/torque/y'])
        plt.plot(tt_ref, tq_y, label='acc', linestyle="-", color=color_ref)
        plt.legend(framealpha=legend_alpha)
        plt.ylabel('$\\tau_{dy}$ (N.m)', fontsize=label_size)

        plt.subplot(3, 2, 6)
        tq_z = np.array(data_dist_acc_torque['/beetle1/disturbance_wrench/wrench/torque/z'])
        plt.plot(tt_ref, tq_z, label='acc', linestyle="-", color=color_ref)
        plt.legend(framealpha=legend_alpha)
        plt.ylabel('$\\tau_{dz}$ (N.m)', fontsize=label_size)

        plt.tight_layout()
        fig.subplots_adjust(hspace=0.2)
        plt.show()

    else:
        print('Invalid type')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Plot the trajectory. Please use plotjuggler to generate the csv file.')
    parser.add_argument('file_path', type=str, help='The file name of the trajectory')
    parser.add_argument('--type', type=int, default=0, help='The type of the trajectory')

    args = parser.parse_args()

    main(args.file_path, args.type)
