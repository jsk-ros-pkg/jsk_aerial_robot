import pandas as pd
import numpy as np
import scienceplots
import matplotlib.pyplot as plt
import argparse

legend_alpha = 0.5

def main(file_path, type):
    # Load the data from csv file
    data = pd.read_csv(file_path)

    if type == 0:
        # ======= xyz =========
        data_xyz_ref = data[
            ['__time', '/beetle1/set_ref_traj/x/data[0]', '/beetle1/set_ref_traj/x/data[1]',
             '/beetle1/set_ref_traj/x/data[2]']]
        data_xyz = data[
            ['__time', '/beetle1/uav/cog/odom/pose/pose/position/x', '/beetle1/uav/cog/odom/pose/pose/position/y',
             '/beetle1/uav/cog/odom/pose/pose/position/z']]
        data_xyz_ref = data_xyz_ref.dropna()
        data_xyz = data_xyz.dropna()

        # ======= rpy =========
        data_qwxyz_ref = data[
            ['__time', '/beetle1/set_ref_traj/x/data[6]', '/beetle1/set_ref_traj/x/data[7]',
             '/beetle1/set_ref_traj/x/data[8]',
             '/beetle1/set_ref_traj/x/data[9]']]
        data_qwxyz = data[
            ['__time', '/beetle1/uav/cog/odom/pose/pose/orientation/w', '/beetle1/uav/cog/odom/pose/pose/orientation/x',
             '/beetle1/uav/cog/odom/pose/pose/orientation/y', '/beetle1/uav/cog/odom/pose/pose/orientation/z']]
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

        # ======= plotting =========
        plt.style.use(["science", "grid"])

        plt.rcParams.update({'font.size': 11})  # default is 10
        label_size = 14

        fig = plt.figure(figsize=(7, 6))

        # --------------------------------
        plt.subplot(3, 2, 1)
        t_ref = np.array(data_xyz_ref['__time'])
        x_ref = np.array(data_xyz_ref['/beetle1/set_ref_traj/x/data[0]'])
        plt.plot(t_ref, x_ref, label='ref')

        t = np.array(data_xyz['__time'])
        x = np.array(data_xyz['/beetle1/uav/cog/odom/pose/pose/position/x'])
        plt.plot(t, x, label='real')

        plt.legend(framealpha=legend_alpha)
        plt.ylabel('X (m)', fontsize=label_size)

        # --------------------------------
        plt.subplot(3, 2, 2)
        t_ref = np.array(data_euler_ref['__time'])
        roll_ref = np.array(data_euler_ref['roll'])
        plt.plot(t_ref, roll_ref, label='ref')

        t = np.array(data_euler['__time'])
        roll = np.array(data_euler['roll'])
        plt.plot(t, roll, label='real')

        plt.legend(framealpha=legend_alpha)
        plt.ylabel('Roll (rad)', fontsize=label_size)

        # --------------------------------
        plt.subplot(3, 2, 3)
        t_ref = np.array(data_xyz_ref['__time'])
        y_ref = np.array(data_xyz_ref['/beetle1/set_ref_traj/x/data[1]'])
        plt.plot(t_ref, y_ref, label='ref')

        t = np.array(data_xyz['__time'])
        y = np.array(data_xyz['/beetle1/uav/cog/odom/pose/pose/position/y'])
        plt.plot(t, y, label='Y')
        plt.ylabel('Y (m)', fontsize=label_size)

        # --------------------------------
        plt.subplot(3, 2, 4)
        t_ref = np.array(data_euler_ref['__time'])
        pitch_ref = np.array(data_euler_ref['pitch'])
        plt.plot(t_ref, pitch_ref, label='ref')

        t = np.array(data_euler['__time'])
        pitch = np.array(data_euler['pitch'])
        plt.plot(t, pitch, label='real')
        plt.ylabel('Pitch (rad)', fontsize=label_size)

        # --------------------------------
        plt.subplot(3, 2, 5)
        t_ref = np.array(data_xyz_ref['__time'])
        z_ref = np.array(data_xyz_ref['/beetle1/set_ref_traj/x/data[2]'])
        plt.plot(t_ref, z_ref, label='ref')

        t = np.array(data_xyz['__time'])
        z = np.array(data_xyz['/beetle1/uav/cog/odom/pose/pose/position/z'])

        plt.plot(t, z, label='Z')
        plt.ylabel('Z (m)', fontsize=label_size)
        plt.xlabel('Time (s)', fontsize=label_size)

        # --------------------------------
        plt.subplot(3, 2, 6)
        t_ref = np.array(data_euler_ref['__time'])
        yaw_ref = np.array(data_euler_ref['yaw'])
        plt.plot(t_ref, yaw_ref, label='ref')

        t = np.array(data_euler['__time'])
        yaw = np.array(data_euler['yaw'])
        plt.plot(t, yaw, label='real')
        plt.ylabel('Yaw (rad)', fontsize=label_size)
        plt.xlabel('Time (s)', fontsize=label_size)

        plt.tight_layout()
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

    else:
        print('Invalid type')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Plot the trajectory')
    parser.add_argument('file_path', type=str, help='The file name of the trajectory')
    parser.add_argument('--type', type=int, help='The type of the trajectory')

    args = parser.parse_args()

    main(args.file_path, args.type)
