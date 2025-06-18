#!/usr/bin/env python
# -*- encoding: ascii -*-

""" 1. read 2. interpolate 3. save """

import argparse
import pandas as pd
import numpy as np


def main(file_path):
    # Load the data from the csv file
    data = pd.read_csv(file_path)

    # servo angle cmd
    data_servo_angle_cmd = data[
        ['__time', '/beetle1/gimbals_ctrl/gimbal1/position', '/beetle1/gimbals_ctrl/gimbal2/position',
         '/beetle1/gimbals_ctrl/gimbal3/position', '/beetle1/gimbals_ctrl/gimbal4/position']]
    data_servo_angle_cmd = data_servo_angle_cmd.dropna()

    # real servo angle
    data_servo_angle = data[
        ['__time', '/beetle1/joint_states/gimbal1/position', '/beetle1/joint_states/gimbal2/position',
         '/beetle1/joint_states/gimbal3/position', '/beetle1/joint_states/gimbal4/position']]
    data_servo_angle = data_servo_angle.dropna()

    # rename columns
    data_servo_angle_cmd.columns = ['__time', 'gimbal1_cmd', 'gimbal2_cmd', 'gimbal3_cmd', 'gimbal4_cmd']
    data_servo_angle.columns = ['__time', 'gimbal1', 'gimbal2', 'gimbal3', 'gimbal4']

    t_bias = min(data_servo_angle_cmd['__time'].min(), data_servo_angle['__time'].min())
    data_servo_angle_cmd['__time'] -= t_bias
    data_servo_angle['__time'] -= t_bias

    # --------------------------------------------------------------------------
    # Build a common 100 Hz time grid on the *overlap* of the two topics
    # --------------------------------------------------------------------------
    freq_inter = 100  # Hz, the frequency of the interpolated data

    time_start = 0.0
    time_end = 4.5

    inter_time = np.arange(time_start, time_end + 1e-6, 1 / freq_inter)

    # Helper: do NumPy interpolation for every column except the time column
    def interp_df(source_df, cols):
        out = {'__time': inter_time}
        for c in cols:
            out[c] = np.interp(
                inter_time,
                source_df['__time'].to_numpy(),
                source_df[c].to_numpy()
            )
        return pd.DataFrame(out)

    # Interpolate
    cmd_cols = ['gimbal1_cmd', 'gimbal2_cmd', 'gimbal3_cmd', 'gimbal4_cmd']
    real_cols = ['gimbal1', 'gimbal2', 'gimbal3', 'gimbal4']

    data_servo_angle_cmd_interp = interp_df(data_servo_angle_cmd, cmd_cols)
    data_servo_angle_interp = interp_df(data_servo_angle, real_cols)

    # save to csv, one is cmd, one is real
    save_path = 'servo_data_cmd_interpolated.csv'
    data_servo_angle_cmd_interp.to_csv(save_path, index=False, header=False)
    save_path = 'servo_data_real_interpolated.csv'
    data_servo_angle_interp.to_csv(save_path, index=False, header=False)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Plot the trajectory. Please use plotjuggler to generate the csv file.')
    parser.add_argument('file_path', type=str, help='The file name of the trajectory')
    args = parser.parse_args()

    main(args.file_path)
