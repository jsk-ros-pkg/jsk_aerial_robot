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
data = pd.read_csv('~/.ros/motor_test_1708236968.txt', sep=' ',
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

# Plotting
plt.style.use(["science", "grid"])
plt.rcParams.update({'font.size': 11})  # default is 10
label_size = 15
legend_alpha = 0.5

# fig = plt.figure(figsize=(3.5, 3))
figure = plt.figure(figsize=(7, 6))
ax = figure.add_subplot(111)
plt.plot(time, cmd, label='PWM')
# plt.plot(time, fz, label='fz')

plt.xlabel('Time (s)', fontsize=label_size)
# plt.ylabel('Servo Angle (rad)', fontsize=label_size)
# plt.title('First-order system response with varying input')
plt.legend(framealpha=legend_alpha, loc='upper left')

ax_right = ax.twinx()
plt.plot(time, rpm, label='RPM', color='red')

plt.legend(framealpha=legend_alpha, loc='upper right')

# plt.xlim([4, 5])
plt.tight_layout()
plt.show()
