'''
created by Jinjie LI, 2024/03/22
'''

import numpy as np
import pandas as pd
from scipy.integrate import odeint
import matplotlib.pyplot as plt
import scienceplots
from scipy.interpolate import interp1d

# Load the input data from the CSV file
# Replace 'path/to/your/file.csv' with the actual file path
input_data = pd.read_csv('20240215-210011_inter_gimbals_ctrl.csv', header=None)
time_input = input_data[0]
u_input = input_data[1]

output_data = pd.read_csv('20240215-210011_inter_joint_states.csv', header=None)
time_output = output_data[0].to_numpy()
y_output = output_data[1].to_numpy()

# Define the time points where you want to solve the ODE
t = np.linspace(min(time_input), max(time_input), len(time_input))

# Interpolate the input data to these time points
u_interpolated = interp1d(time_input, u_input, kind='linear', fill_value='extrapolate')


# Define the first-order differential equation using the interpolated input
def model(y, t):
    tau = 0.08
    u = u_interpolated(t)
    dydt = (1 / tau) * (u - y)
    return dydt


# Initial condition
y0 = 0

# Solve ODE
y = odeint(model, y0, t)

# Plotting
plt.style.use(["science", "grid"])
plt.rcParams.update({'font.size': 11})  # default is 10
label_size = 15
legend_alpha = 0.5

fig = plt.figure(figsize=(3.5, 3))
plt.plot(t, u_interpolated(t), label='input')
plt.plot(time_output, y_output, label='real')
plt.plot(t, y, label='estimated')
plt.xlabel('Time (s)', fontsize=label_size)
plt.ylabel('Servo Angle (rad)', fontsize=label_size)
# plt.title('First-order system response with varying input')
plt.legend(framealpha=legend_alpha)
plt.xlim([4, 5])
plt.tight_layout()
plt.show()
