'''
created by Jinjie LI, 2024/03/22
'''

import numpy as np
import pandas as pd
from scipy.integrate import odeint
import matplotlib.pyplot as plt
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
plt.figure(figsize=(10, 5))
plt.plot(t, y, label='y(t)')
plt.plot(t, u_interpolated(t), 'r--', label='Interpolated input u(t)')
plt.plot(time_output, y_output, 'g--', label='Actual output y(t)')
plt.xlabel('Time')
plt.ylabel('Values')
plt.title('First-order system response with varying input')
plt.legend()
plt.grid()
plt.show()
