'''
created by Jinjie LI, 2023/02/06
'''

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.fft import fft
import argparse


def read_data(file_name):
    # Read the data, assuming space-separated values and the first row as header
    data = pd.read_csv(file_name, delim_whitespace=True, header=None)
    # Assign column names based on the number of columns
    col_names = ['time'] + [f'col_{i}' for i in range(1, data.shape[1])]
    data.columns = col_names
    return data


def process_data(data, L, D, m, g):
    # Determine the number of subplots needed: 2 plots per data column, excluding 'time'
    num_cols = data.shape[1] - 1
    fig, axs = plt.subplots(num_cols, 2, figsize=(25, num_cols * 5))

    # Adjust layout for better readability
    plt.tight_layout(pad=3.0)

    # Drop the time column for visualization
    data_for_vis = data.drop(columns=['time'])

    if num_cols == 6:
        name_list = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
    else:
        name_list = ['x', 'y', 'z', 'qw', 'qx', 'qy', 'qz']

    # convert "time" data to PeriodIndex
    data['time'] = pd.to_datetime(data['time'], unit='s')

    # Interpolation to 100hz
    data = data.set_index('time').resample('10ms').mean().interpolate().reset_index()
    average_interval = 0.01

    for i, col in enumerate(data_for_vis.columns):
        # XY plot
        axs[i, 0].plot(data['time'].to_numpy(), data[col].to_numpy())
        axs[i, 0].set_title(f'XY Plot of {name_list[i]}')
        axs[i, 0].set_xlabel('Time (s)')
        axs[i, 0].set_ylabel(name_list[i])

        # FFT transformation
        yf = fft(data[col].to_numpy() - np.mean(data[col].to_numpy()))  # Remove the DC component

        xf = np.linspace(0.0, 1.0 / (2.0 * average_interval), len(data[col]) // 2)
        axs[i, 1].plot(xf, 2.0 / len(data[col]) * np.abs(yf[:len(data[col]) // 2]))
        axs[i, 1].set_title(f'FFT of {name_list[i]}')
        axs[i, 1].set_xlabel('Frequency')
        axs[i, 1].set_ylabel('Amplitude')

        # Find and plot the peak
        peak_freq = xf[np.argmax(np.abs(yf[:len(data[col]) // 2]))]
        peak_amp = 2.0 / len(data[col]) * np.max(np.abs(yf[:len(data[col]) // 2]))
        print(f'- Peak frequency for {name_list[i]}: {peak_freq:.8f} Hz; Peak amplitude: {peak_amp:.8f}')
        axs[i, 1].plot(peak_freq, peak_amp, 'r*', markersize=10)
        axs[i, 1].text(peak_freq, peak_amp, f'Peak: {peak_freq:.8f} Hz, {peak_amp:.8f}', ha='left', va='bottom')

        # Calculate the inertial
        I = m * g * (D ** 2) / (16 * (np.pi ** 2) * L * peak_freq ** 2)
        print(f"Calculate inertial parameters with {name_list[i]}: {I} kg*m^2 \n")

    plt.show()
    print("+++++ Please choose a proper inertial parameter from the above results. +++++")
    print("+++++ Note that the Peak amplitude should be large enough. +++++")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Process and visualize data from a text file.')
    parser.add_argument('file_name', help='Name of the text file containing the data')
    parser.add_argument('L', help='Length of the pendulum', default=1.920)
    parser.add_argument('D', help='Diameter of the pendulum', default=0.495)
    parser.add_argument('m', help='Mass of the object', default=2.773)
    parser.add_argument('--g', help='Acceleration due to gravity', default=9.798)
    args = parser.parse_args()

    data = read_data(args.file_name)
    process_data(data, float(args.L), float(args.D), float(args.m), float(args.g))
