import pandas as pd
import numpy as np
import scienceplots  # For better plotting styles
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt
import argparse

legend_alpha = 0.5

matlab_blue = '#0072BD'
matlab_orange = '#D95319'
matlab_yellow = '#EDB120'
matlab_green = '#77AC30'
matlab_purple = "#7E2F8E"


def unwrap_angle_sequence(angle_seq: np.ndarray) -> np.ndarray:
    angle_seq = angle_seq.copy()  # avoid modifying the input array
    for i in range(1, len(angle_seq)):
        delta = angle_seq[i] - angle_seq[i - 1]
        if delta > np.pi:
            angle_seq[i:] -= 2 * np.pi
        elif delta < -np.pi:
            angle_seq[i:] += 2 * np.pi
    return angle_seq


def calculate_rmse(t, x, t_ref, x_ref, is_yaw=False):
    x_ref_interp = np.interp(t, t_ref, x_ref)
    if is_yaw:
        # calculate the RMSE for yaw
        error = np.minimum(np.abs(x - x_ref_interp), 2 * np.pi - np.abs(x - x_ref_interp))
    else:
        error = x - x_ref_interp

    rmse_x = np.sqrt(np.mean(error ** 2))
    return rmse_x


def quat2euler(qw: pd.Series,
               qx: pd.Series,
               qy: pd.Series,
               qz: pd.Series,
               sequence: str = "ZYX",
               degrees: bool = True) -> tuple[pd.Series, pd.Series, pd.Series]:
    """
    Convert quaternion (w, x, y, z) series to Euler angles.

    Parameters
    ----------
    qw, qx, qy, qz : pandas.Series
        Aligned quaternion components (same index, same length).
    sequence : str, default "xyz"
        Axis sequence for Euler output.  Common choices:
        "xyz"  → roll-pitch-yaw,
        "zyx"  → yaw-pitch-roll, etc.
    degrees : bool, default True
        Return angles in degrees (True) or radians (False).

    Returns
    -------
    roll, pitch, yaw : pandas.Series
        Euler angles in the requested unit, sharing the original index.
    """
    # Stack into the (N, 4) format expected by scipy (x, y, z, w)
    quat_array = np.column_stack([qx.to_numpy(),
                                  qy.to_numpy(),
                                  qz.to_numpy(),
                                  qw.to_numpy()])

    # Convert
    euler = Rotation.from_quat(quat_array).as_euler(sequence, degrees=degrees)

    # Wrap back into Series with the original time index
    idx = qw.index
    roll = pd.Series(euler[:, 0], index=idx, name="roll")
    pitch = pd.Series(euler[:, 1], index=idx, name="pitch")
    yaw = pd.Series(euler[:, 2], index=idx, name="yaw")
    return roll, pitch, yaw


# ---------- Helper: plot reference, robot and error in one subplot --------
def _plot_with_error(ax, t, y, t_ref, y_ref, y_label_left, y_label_right, label_size, time_start_rotate,
                     time_stop_rotate, has_legend=False, legend_loc='center left'):
    """Draw reference (hand), robot and |error| on the same time axis.

    Parameters
    ----------
    ax : matplotlib.axes.Axes
        Left-hand y-axis for reference & robot.
    t, y : 1-D array
        Time and robot signal.
    t_ref, y_ref : 1-D array
        Time and reference (hand) signal.
    y_label_left : str
        Label for the left y-axis.
    """
    # Plot the time span of the rotation. This should be plotted before the main curves
    ax.axvspan(time_start_rotate, time_stop_rotate,
               facecolor=matlab_yellow, alpha=0.3)

    # Secondary y-axis for error
    ax_err = ax.twinx()

    # Right y-axis: absolute error
    abs_err = np.abs(y - np.interp(t, t_ref, y_ref))
    ln3 = ax_err.fill_between(
        t, 0, abs_err,
        color=matlab_blue,
        alpha=0.2,
        label="error"
    )

    # Left y-axis curves
    ln1 = ax.plot(t_ref, y_ref, linestyle="--", color=matlab_blue,
                  label='hand (offset)')[0]
    ln2 = ax.plot(t, y, linestyle="-", color=matlab_orange,
                  label='robot')[0]

    # Axis labels
    ax.set_ylabel(y_label_left, fontsize=label_size)
    ax_err.set_ylabel(y_label_right, fontsize=label_size, rotation=90, labelpad=0)

    # Formatting
    ax.set_xlim(0, 50)
    ax.grid(True)

    # Merge legends (handles from both y-axes)
    handles = [ln1, ln2, ln3]
    if has_legend:
        ax.legend(handles, [h.get_label() for h in handles],
                  framealpha=legend_alpha, ncol=3, loc=legend_loc, fontsize=label_size - 1)

    return abs_err  # You can compute RMSE outside if needed


def main(file_path, plot_type):
    # Load the data from csv file
    data = pd.read_csv(file_path)

    if plot_type == 0:
        # ====== Data acquisition (same as original) ==============================
        # --- xyz ---
        data_xyz = data[['__time',
                         '/beetle1/uav/cog/odom/pose/pose/position/x',
                         '/beetle1/uav/cog/odom/pose/pose/position/y',
                         '/beetle1/uav/cog/odom/pose/pose/position/z']].dropna()
        data_xyz_ref = data[['__time',
                             '/hand/mocap/pose/pose/position/x',
                             '/hand/mocap/pose/pose/position/y',
                             '/hand/mocap/pose/pose/position/z']].dropna()

        # --- quaternion ---
        data_qwxyz = data[['__time',
                           '/beetle1/uav/cog/odom/pose/pose/orientation/w',
                           '/beetle1/uav/cog/odom/pose/pose/orientation/x',
                           '/beetle1/uav/cog/odom/pose/pose/orientation/y',
                           '/beetle1/uav/cog/odom/pose/pose/orientation/z']].dropna()
        data_qwxyz_ref = data[['__time',
                               '/hand/mocap/pose/pose/orientation/w',
                               '/hand/mocap/pose/pose/orientation/x',
                               '/hand/mocap/pose/pose/orientation/y',
                               '/hand/mocap/pose/pose/orientation/z']].dropna()

        # ====== Plot settings =====================================================
        plt.style.use(["science", "grid"])
        plt.rcParams.update({'font.size': 11})
        label_size = 14
        t_bias = max(data_xyz['__time'].iloc[0], data_xyz_ref['__time'].iloc[0])
        time_start_rotate = 28.6993
        time_stop_rotate = 35.2177

        fig, axes = plt.subplots(7, 1, sharex=True, figsize=(7, 6))

        # ---------- 1. p_x --------------------------------------------------------
        t = np.array(data_xyz['__time']) - t_bias
        x = np.array(data_xyz['/beetle1/uav/cog/odom/pose/pose/position/x'])
        t_ref = np.array(data_xyz_ref['__time']) - t_bias
        x_ref = np.array(data_xyz_ref['/hand/mocap/pose/pose/position/x']) \
                - data_xyz_ref['/hand/mocap/pose/pose/position/x'].iloc[0] + x[0]
        _plot_with_error(axes[0], t, x, t_ref, x_ref, '$p_x$ [m]', '$e_{p_x}$ [m]', label_size, time_start_rotate,
                         time_stop_rotate,
                         has_legend=True)

        # ---------- 2. p_y --------------------------------------------------------
        y = np.array(data_xyz['/beetle1/uav/cog/odom/pose/pose/position/y'])
        y_ref = np.array(data_xyz_ref['/hand/mocap/pose/pose/position/y']) \
                - data_xyz_ref['/hand/mocap/pose/pose/position/y'].iloc[0] + y[0]
        _plot_with_error(axes[1], t, y, t_ref, y_ref, '$p_y$ [m]', '$e_{p_y}$ [m]', label_size, time_start_rotate,
                         time_stop_rotate)

        # ---------- 3. p_z --------------------------------------------------------
        z = np.array(data_xyz['/beetle1/uav/cog/odom/pose/pose/position/z'])
        z_ref = np.array(data_xyz_ref['/hand/mocap/pose/pose/position/z'])
        _plot_with_error(axes[2], t, z, t_ref, z_ref, '$p_z$ [m]', '$e_{p_z}$ [m]', label_size, time_start_rotate,
                         time_stop_rotate)

        # ---------- 4. q_w --------------------------------------------------------
        qw = np.array(data_qwxyz['/beetle1/uav/cog/odom/pose/pose/orientation/w'])
        qw_ref = -np.array(data_qwxyz_ref['/hand/mocap/pose/pose/orientation/w'])
        t_q = np.array(data_qwxyz['__time']) - t_bias
        t_qref = np.array(data_qwxyz_ref['__time']) - t_bias
        _plot_with_error(axes[3], t_q, qw, t_qref, qw_ref, '$q_w$', '$e_{q_w}$', label_size, time_start_rotate,
                         time_stop_rotate)

        # ---------- 5. q_x --------------------------------------------------------
        qx = np.array(data_qwxyz['/beetle1/uav/cog/odom/pose/pose/orientation/x'])
        qx_ref = -np.array(data_qwxyz_ref['/hand/mocap/pose/pose/orientation/x'])
        _plot_with_error(axes[4], t_q, qx, t_qref, qx_ref, '$q_x$', '$e_{q_x}$', label_size, time_start_rotate,
                         time_stop_rotate, has_legend=False)

        # ---------- 6. q_y --------------------------------------------------------
        qy = np.array(data_qwxyz['/beetle1/uav/cog/odom/pose/pose/orientation/y'])
        qy_ref = -np.array(data_qwxyz_ref['/hand/mocap/pose/pose/orientation/y'])
        _plot_with_error(axes[5], t_q, qy, t_qref, qy_ref, '$q_y$', '$e_{q_y}$', label_size, time_start_rotate,
                         time_stop_rotate)

        # ---------- 7. q_z --------------------------------------------------------
        qz = np.array(data_qwxyz['/beetle1/uav/cog/odom/pose/pose/orientation/z'])
        qz_ref = -np.array(data_qwxyz_ref['/hand/mocap/pose/pose/orientation/z'])
        _plot_with_error(axes[6], t_q, qz, t_qref, qz_ref, '$q_z$', '$e_{q_z}$', label_size, time_start_rotate,
                         time_stop_rotate)

        # ---------- Shared X-axis -------------------------------------------------
        for ax in axes[:-1]:
            ax.tick_params(labelbottom=False)
        axes[-1].set_xlabel('Time $t$ [s]', fontsize=label_size)

        plt.tight_layout()
        fig.subplots_adjust(hspace=0.25)
        plt.show()

    if plot_type == 2:
        # ======= xyz =========
        data_xyz = data[
            ['__time', '/beetle1/uav/cog/odom/pose/pose/position/x', '/beetle1/uav/cog/odom/pose/pose/position/y',
             '/beetle1/uav/cog/odom/pose/pose/position/z']].dropna()

        data_xyz_ref = data[
            ['__time', '/hand/mocap/pose/pose/position/x', '/hand/mocap/pose/pose/position/y',
             '/hand/mocap/pose/pose/position/z']].dropna()

        # ======= rpy =========
        data_qwxyz = data[
            ['__time', '/beetle1/uav/cog/odom/pose/pose/orientation/w', '/beetle1/uav/cog/odom/pose/pose/orientation/x',
             '/beetle1/uav/cog/odom/pose/pose/orientation/y', '/beetle1/uav/cog/odom/pose/pose/orientation/z']].dropna()

        data_qwxyz_ref = data[
            ['__time', '/hand/mocap/pose/pose/orientation/w', '/hand/mocap/pose/pose/orientation/x',
             '/hand/mocap/pose/pose/orientation/y', '/hand/mocap/pose/pose/orientation/z']].dropna()

        # convert to euler
        data_euler_ref = pd.DataFrame()
        data_euler_ref['__time'] = data_qwxyz_ref['__time']
        data_euler_ref = pd.DataFrame()
        data_euler_ref['__time'] = data_qwxyz_ref['__time']
        data_euler_ref['roll'], data_euler_ref['pitch'], data_euler_ref['yaw'] = quat2euler(
            data_qwxyz_ref['/hand/mocap/pose/pose/orientation/w'],
            data_qwxyz_ref['/hand/mocap/pose/pose/orientation/x'],
            data_qwxyz_ref['/hand/mocap/pose/pose/orientation/y'],
            data_qwxyz_ref['/hand/mocap/pose/pose/orientation/z'])

        data_euler = pd.DataFrame()
        data_euler['__time'] = data_qwxyz['__time']
        data_euler['roll'], data_euler['pitch'], data_euler['yaw'] = quat2euler(
            data_qwxyz['/beetle1/uav/cog/odom/pose/pose/orientation/w'],
            data_qwxyz['/beetle1/uav/cog/odom/pose/pose/orientation/x'],
            data_qwxyz['/beetle1/uav/cog/odom/pose/pose/orientation/y'],
            data_qwxyz['/beetle1/uav/cog/odom/pose/pose/orientation/z'])

        # ======= Plotting Settings =======
        plt.style.use(["science", "grid"])
        plt.rcParams.update({'font.size': 11})
        label_size = 14

        # The later initial time of the two data is used as the time offset
        t_bias = max(data_xyz['__time'].iloc[0], data_xyz_ref['__time'].iloc[0])
        # Define the rotation interval (only for annotation in the figure)
        time_start_rotate = 28.6993
        time_stop_rotate = 35.2177

        # Create 7 subplots with a shared x-axis
        fig, axes = plt.subplots(6, 1, sharex=True, figsize=(7, 6))

        # --- Subplot 1: X position ---
        ax = axes[0]
        t_ref = np.array(data_xyz_ref['__time']) - t_bias
        x_ref = np.array(data_xyz_ref['/hand/mocap/pose/pose/position/x'])
        t = np.array(data_xyz['__time']) - t_bias
        x = np.array(data_xyz['/beetle1/uav/cog/odom/pose/pose/position/x'])
        x_ref_offset = x_ref - x_ref[0] + x[0]

        ax.plot(t_ref, x_ref, label='hand', linestyle="--", color=matlab_blue)
        ax.plot(t_ref, x_ref_offset, label='hand offset', linestyle="-.", color="k")
        ax.plot(t, x, label='robot', color=matlab_orange)
        ax.legend(framealpha=legend_alpha, loc="center right")
        ax.set_ylabel('X [m]', fontsize=label_size)
        ax.set_xlim(0, 50)
        ax.axvspan(time_start_rotate, time_stop_rotate, facecolor="#EDB120", alpha=0.3)

        rmse_x = calculate_rmse(t, x, t_ref, x_ref)
        print(f'RMSE X [m]: {rmse_x}')

        # --- Subplot 2: Y position ---
        ax = axes[1]
        t_ref = np.array(data_xyz_ref['__time']) - t_bias
        y_ref = np.array(data_xyz_ref['/hand/mocap/pose/pose/position/y'])
        t = np.array(data_xyz['__time']) - t_bias
        y = np.array(data_xyz['/beetle1/uav/cog/odom/pose/pose/position/y'])
        y_ref_offset = y_ref - y_ref[0] + y[0]

        ax.plot(t_ref, y_ref, label='hand', linestyle="--", color=matlab_blue)
        ax.plot(t_ref, y_ref_offset, label='hand offset', linestyle="-.", color="k")
        ax.plot(t, y, label='robot', color=matlab_orange)
        ax.set_ylabel('Y [m]', fontsize=label_size)
        ax.set_xlim(0, 50)
        ax.axvspan(time_start_rotate, time_stop_rotate, facecolor="#EDB120", alpha=0.3)

        rmse_y = calculate_rmse(t, y, t_ref, y_ref)
        print(f'RMSE Y [m]: {rmse_y}')

        # --- Subplot 3: Z position ---
        ax = axes[2]
        t_ref = np.array(data_xyz_ref['__time']) - t_bias
        z_ref = np.array(data_xyz_ref['/hand/mocap/pose/pose/position/z'])
        ax.plot(t_ref, z_ref, label='hand', linestyle="--", color=matlab_blue)
        t = np.array(data_xyz['__time']) - t_bias
        z = np.array(data_xyz['/beetle1/uav/cog/odom/pose/pose/position/z'])
        ax.plot(t, z, label='robot', color=matlab_orange)
        ax.set_ylabel('Z [m]', fontsize=label_size)
        ax.set_xlim(0, 50)
        ax.axvspan(time_start_rotate, time_stop_rotate, facecolor="#EDB120", alpha=0.3)

        rmse_z = calculate_rmse(t, z, t_ref, z_ref)
        print(f'RMSE Z [m]: {rmse_z}')

        # --- Subplot 4: Roll ---
        ax = axes[3]
        t_ref = np.array(data_euler_ref['__time']) - t_bias
        roll_ref = unwrap_angle_sequence(np.array(data_euler_ref['roll']))
        ax.plot(t_ref, roll_ref, label='hand', linestyle="--", color=matlab_blue)
        t = np.array(data_euler['__time']) - t_bias
        roll = unwrap_angle_sequence(np.array(data_euler['roll']))
        ax.plot(t, roll, label='robot', color=matlab_orange)
        ax.legend(framealpha=legend_alpha, loc='lower right')
        ax.set_ylabel('Roll [$^\circ$]', fontsize=label_size)
        ax.set_xlim(0, 50)
        ax.axvspan(time_start_rotate, time_stop_rotate, facecolor="#EDB120", alpha=0.3)
        rmse_roll = calculate_rmse(t, roll, t_ref, roll_ref)
        print(f'RMSE Roll [rad]: {rmse_roll}')

        # --- Subplot 5: Pitch ---
        ax = axes[4]
        t_ref = np.array(data_euler_ref['__time']) - t_bias
        pitch_ref = unwrap_angle_sequence(np.array(data_euler_ref['pitch']))
        ax.plot(t_ref, pitch_ref, label='hand', linestyle="--", color=matlab_blue)
        t = np.array(data_euler['__time']) - t_bias
        pitch = unwrap_angle_sequence(np.array(data_euler['pitch']))
        ax.plot(t, pitch, label='robot', color=matlab_orange)
        ax.set_ylabel('Pitch [$^\circ$]', fontsize=label_size)
        ax.set_xlim(0, 50)
        ax.axvspan(time_start_rotate, time_stop_rotate, facecolor="#EDB120", alpha=0.3)
        rmse_pitch = calculate_rmse(t, pitch, t_ref, pitch_ref)
        print(f'RMSE Pitch [rad]: {rmse_pitch}')

        # --- Subplot 6: Yaw ---
        ax = axes[5]
        t_ref = np.array(data_euler_ref['__time']) - t_bias
        yaw_ref = unwrap_angle_sequence(np.array(data_euler_ref['yaw']))
        ax.plot(t_ref, yaw_ref, label='hand', linestyle="--", color=matlab_blue)
        t = np.array(data_euler['__time']) - t_bias
        yaw = unwrap_angle_sequence(np.array(data_euler['yaw']))
        ax.plot(t, yaw, label='robot', color=matlab_orange)
        ax.set_ylabel('Yaw [$^\circ$]', fontsize=label_size)
        ax.set_xlim(0, 50)
        ax.axvspan(time_start_rotate, time_stop_rotate, facecolor="#EDB120", alpha=0.3)
        ax.legend(framealpha=legend_alpha, loc='lower right')
        rmse_yaw = calculate_rmse(t, yaw, t_ref, yaw_ref, is_yaw=True)
        print(f'RMSE Yaw [rad]: {rmse_yaw}')

        # --- Hide the X-axis scales of all subplots except the bottom one, and set a common X-axis label ---
        for ax in axes[:-1]:
            ax.tick_params(labelbottom=False)
        axes[-1].set_xlabel('Time [s]', fontsize=label_size)

        plt.tight_layout()
        fig.subplots_adjust(hspace=0.2)
        plt.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Plot the trajectory. Please use plotjuggler to generate the csv file.')
    parser.add_argument('file_path', type=str, help='The file name of the trajectory')
    parser.add_argument('--type', type=int, default=0, help='The type of plotting')

    args = parser.parse_args()

    main(args.file_path, args.type)
