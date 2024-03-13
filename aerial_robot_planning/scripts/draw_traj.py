import pandas as pd
import numpy as np
import scienceplots
import matplotlib.pyplot as plt

# Load the data from csv file
data = pd.read_csv('/home/jinjie/Desktop/SetPoint.csv')

# ======= xyz =========
data_xyz_ref = data[
    ['__time', '/beetle1/set_ref_traj/x/data[0]', '/beetle1/set_ref_traj/x/data[1]', '/beetle1/set_ref_traj/x/data[2]']]
data_xyz = data[['__time', '/beetle1/uav/cog/odom/pose/pose/position/x', '/beetle1/uav/cog/odom/pose/pose/position/y',
                 '/beetle1/uav/cog/odom/pose/pose/position/z']]
data_xyz_ref = data_xyz_ref.dropna()
data_xyz = data_xyz.dropna()

# ======= rpy =========
data_qwxyz_ref = data[
    ['__time', '/beetle1/set_ref_traj/x/data[6]', '/beetle1/set_ref_traj/x/data[7]', '/beetle1/set_ref_traj/x/data[8]',
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

plt.legend()
plt.ylabel('X (m)')

# --------------------------------
plt.subplot(3, 2, 2)
t_ref = np.array(data_euler_ref['__time'])
roll_ref = np.array(data_euler_ref['roll'])
plt.plot(t_ref, roll_ref, label='ref')

t = np.array(data_euler['__time'])
roll = np.array(data_euler['roll'])
plt.plot(t, roll, label='real')

plt.legend()
plt.ylabel('Roll (rad)')

# --------------------------------
plt.subplot(3, 2, 3)
t_ref = np.array(data_xyz_ref['__time'])
y_ref = np.array(data_xyz_ref['/beetle1/set_ref_traj/x/data[1]'])
plt.plot(t_ref, y_ref, label='ref')

t = np.array(data_xyz['__time'])
y = np.array(data_xyz['/beetle1/uav/cog/odom/pose/pose/position/y'])
plt.plot(t, y, label='Y')
plt.ylabel('Y (m)')

# --------------------------------
plt.subplot(3, 2, 4)
t_ref = np.array(data_euler_ref['__time'])
pitch_ref = np.array(data_euler_ref['pitch'])
plt.plot(t_ref, pitch_ref, label='ref')

t = np.array(data_euler['__time'])
pitch = np.array(data_euler['pitch'])
plt.plot(t, pitch, label='real')
plt.ylabel('Pitch (rad)')

# --------------------------------
plt.subplot(3, 2, 5)
t_ref = np.array(data_xyz_ref['__time'])
z_ref = np.array(data_xyz_ref['/beetle1/set_ref_traj/x/data[2]'])
plt.plot(t_ref, z_ref, label='ref')

t = np.array(data_xyz['__time'])
z = np.array(data_xyz['/beetle1/uav/cog/odom/pose/pose/position/z'])

plt.plot(t, z, label='Z')
plt.ylabel('Z (m)')
plt.xlabel('Time (s)')

# --------------------------------
plt.subplot(3, 2, 6)
t_ref = np.array(data_euler_ref['__time'])
yaw_ref = np.array(data_euler_ref['yaw'])
plt.plot(t_ref, yaw_ref, label='ref')

t = np.array(data_euler['__time'])
yaw = np.array(data_euler['yaw'])
plt.plot(t, yaw, label='real')
plt.ylabel('Yaw (rad)')
plt.xlabel('Time (s)')

plt.tight_layout()
plt.show()
