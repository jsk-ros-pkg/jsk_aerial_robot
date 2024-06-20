'''
created by Jinjie LI, 2024/03/23
'''

import numpy as np
import pandas as pd
from scipy.integrate import odeint
import matplotlib.pyplot as plt
import scienceplots
from scipy.interpolate import interp1d

# Load the input data from the CSV file
data = pd.read_csv('~/.ros/servo=0_u=23.2v_motor_test_1718360916.txt', sep=' ',
                   names=["PWM", "fx", "fy", "fz", "f_norm", "mx", "my", "mz", "currency", "RPM", "temperature",
                          "voltage", "State"])

data = data[data['PWM'] != 'done']
data['PWM'] = data['PWM'].astype(int)

freq = 50  # Hz
time = np.linspace(0, len(data) / freq, len(data))
cmd = (data['PWM'] - 1000).to_numpy()
cmd_norm = cmd / max(cmd)
fz = data['fz'].to_numpy()
fz_norm = fz / max(fz)
rpm = data['RPM'].to_numpy()
rpm_norm = rpm / max(rpm)

# thrust_cmd has the same shape with cmd, but new values
thrust_cmd = np.zeros(len(cmd))
thrust_cmd[cmd > 650] = 14

# Interpolate the input data to these time points
u_interpolated = interp1d(time, thrust_cmd, kind='linear', fill_value='extrapolate')


# Define the first-order differential equation using the interpolated input
def model(y, t):
    tau = 0.0942
    delay = 0.35
    u = u_interpolated(t - delay)
    dydt = (1 / tau) * (u - y)
    return dydt


# Initial condition
y0 = 0

# Solve ODE
y = odeint(model, y0, time)

# Plotting
plt.style.use(["science", "grid"])
plt.rcParams.update({'font.size': 11})  # default is 10
label_size = 15
legend_alpha = 0.5

# fig = plt.figure(figsize=(3.5, 3))
figure = plt.figure(figsize=(3.5, 2.5))

plt.plot(time, thrust_cmd, label='input')
plt.plot(time, fz, label='real')
plt.plot(time, y, label='estimated')

plt.xlabel('Time (s)', fontsize=label_size)
plt.xlim([200, 201.5])

plt.ylabel('Thrust (N)', fontsize=label_size)
plt.ylim([-2, 16])

plt.legend(framealpha=legend_alpha, loc='lower right')

plt.tight_layout()
plt.show()
