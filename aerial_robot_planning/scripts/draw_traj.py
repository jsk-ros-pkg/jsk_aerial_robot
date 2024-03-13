import pandas as pd
import numpy as np
import scienceplots
import matplotlib.pyplot as plt

# Load the data from csv file
data = pd.read_csv('/home/jinjie/Desktop/SetPoint.csv')

data_xyz_ref = data[
    ['__time', '/beetle1/set_ref_traj/x/data[0]', '/beetle1/set_ref_traj/x/data[1]', '/beetle1/set_ref_traj/x/data[2]']]
data_xyz = data[['__time', '/beetle1/uav/cog/odom/pose/pose/position/x', '/beetle1/uav/cog/odom/pose/pose/position/y',
                 '/beetle1/uav/cog/odom/pose/pose/position/z']]
data_xyz_ref = data_xyz_ref.dropna()
data_xyz = data_xyz.dropna()

plt.style.use(["science", "grid"])

plt.rcParams.update({'font.size': 11})  # default is 10
label_size = 14

fig = plt.figure(figsize=(7, 4))

plt.subplot(3, 2, 1)
t_ref = np.array(data_xyz_ref['__time'])
x_ref = np.array(data_xyz_ref['/beetle1/set_ref_traj/x/data[0]'])
plt.plot(t_ref, x_ref, label='ref')

t = np.array(data_xyz['__time'])
x = np.array(data_xyz['/beetle1/uav/cog/odom/pose/pose/position/x'])
plt.plot(t, x, label='real')

plt.legend()
plt.ylabel('X (m)')

plt.subplot(3, 2, 3)
t_ref = np.array(data_xyz_ref['__time'])
y_ref = np.array(data_xyz_ref['/beetle1/set_ref_traj/x/data[1]'])
plt.plot(t_ref, y_ref, label='ref')

t = np.array(data_xyz['__time'])
y = np.array(data_xyz['/beetle1/uav/cog/odom/pose/pose/position/y'])
plt.plot(t, y, label='Y')
plt.ylabel('Y (m)')

plt.subplot(3, 2, 5)
t_ref = np.array(data_xyz_ref['__time'])
z_ref = np.array(data_xyz_ref['/beetle1/set_ref_traj/x/data[2]'])
plt.plot(t_ref, z_ref, label='ref')

t = np.array(data_xyz['__time'])
z = np.array(data_xyz['/beetle1/uav/cog/odom/pose/pose/position/z'])

plt.plot(t, z, label='Z')
plt.ylabel('Z (m)')
plt.xlabel('Time (s)')

plt.tight_layout()
plt.show()
