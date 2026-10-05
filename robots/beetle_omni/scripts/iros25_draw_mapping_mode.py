import pandas as pd
import numpy as np
import scienceplots  # For better plotting styles
import matplotlib.pyplot as plt
import argparse

from utils import quat2euler, calculate_rmse, calculate_quat_error, unwrap_angle_sequence
from utils import matlab_blue, matlab_orange, matlab_yellow

legend_alpha = 0.5


# ---------- Helper: plot reference, robot and error in one subplot --------
def _plot_with_error(
    ax,
    t,
    y,
    t_ref,
    y_ref,
    y_label_left,
    y_label_right,
    label_size,
    time_start_rotate,
    time_stop_rotate,
    has_legend=False,
    legend_loc="center left",
):
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
    ax.axvspan(time_start_rotate, time_stop_rotate, facecolor=matlab_yellow, alpha=0.3)

    # Secondary y-axis for error
    ax_err = ax.twinx()

    # Right y-axis: absolute error
    abs_err = np.abs(y - np.interp(t, t_ref, y_ref))
    ln3 = ax_err.fill_between(t, 0, abs_err, color=matlab_blue, alpha=0.2, label="error")

    # Left y-axis curves
    ln1 = ax.plot(t_ref, y_ref, linestyle="--", color=matlab_blue, label="hand (offset)")[0]
    ln2 = ax.plot(t, y, linestyle="-", color=matlab_orange, label="robot")[0]

    # Axis labels
    ax.set_ylabel(y_label_left, fontsize=label_size)
    ax_err.set_ylabel(y_label_right, fontsize=label_size, rotation=90, labelpad=5)

    # Formatting
    ax.set_xlim(0, 50)
    ax.grid(True)

    # Merge legends (handles from both y-axes)
    handles = [ln1, ln2, ln3]
    if has_legend:
        ax.legend(
            handles,
            [h.get_label() for h in handles],
            framealpha=legend_alpha,
            ncol=3,
            loc=legend_loc,
            fontsize=label_size - 1,
        )

    return abs_err  # You can compute RMSE outside if needed


def main(file_path, plot_type):
    # Load the data from csv file
    data = pd.read_csv(file_path)

    if plot_type == 0:
        # ====== Data acquisition (same as original) ==============================
        # --- xyz ---
        data_xyz = data[
            [
                "__time",
                "/beetle1/uav/cog/odom/pose/pose/position/x",
                "/beetle1/uav/cog/odom/pose/pose/position/y",
                "/beetle1/uav/cog/odom/pose/pose/position/z",
            ]
        ].dropna()
        data_xyz_ref = data[
            [
                "__time",
                "/hand/mocap/pose/pose/position/x",
                "/hand/mocap/pose/pose/position/y",
                "/hand/mocap/pose/pose/position/z",
            ]
        ].dropna()

        # --- quaternion ---
        data_qwxyz = data[
            [
                "__time",
                "/beetle1/uav/cog/odom/pose/pose/orientation/w",
                "/beetle1/uav/cog/odom/pose/pose/orientation/x",
                "/beetle1/uav/cog/odom/pose/pose/orientation/y",
                "/beetle1/uav/cog/odom/pose/pose/orientation/z",
            ]
        ].dropna()
        data_qwxyz_ref = data[
            [
                "__time",
                "/hand/mocap/pose/pose/orientation/w",
                "/hand/mocap/pose/pose/orientation/x",
                "/hand/mocap/pose/pose/orientation/y",
                "/hand/mocap/pose/pose/orientation/z",
            ]
        ].dropna()

        # ====== Plot settings =====================================================
        plt.style.use(["science", "grid"])
        plt.rcParams.update({"font.size": 11})
        label_size = 14
        t_bias = max(data_xyz["__time"].iloc[0], data_xyz_ref["__time"].iloc[0])
        time_start_rotate = 28.6993
        time_stop_rotate = 35.2177

        fig, axes = plt.subplots(7, 1, sharex=True, figsize=(7, 6))

        # ---------- 1. p_x --------------------------------------------------------
        t = np.array(data_xyz["__time"]) - t_bias
        x = np.array(data_xyz["/beetle1/uav/cog/odom/pose/pose/position/x"])
        t_ref = np.array(data_xyz_ref["__time"]) - t_bias
        x_ref = (
            np.array(data_xyz_ref["/hand/mocap/pose/pose/position/x"])
            - data_xyz_ref["/hand/mocap/pose/pose/position/x"].iloc[0]
            + x[0]
        )
        _plot_with_error(
            axes[0],
            t,
            x,
            t_ref,
            x_ref,
            "$p_x$ [m]",
            "$\\lvert e_{p_x} \\rvert$ [m]",
            label_size,
            time_start_rotate,
            time_stop_rotate,
            has_legend=True,
        )

        rmse_x = calculate_rmse(t, x, t_ref, x_ref)
        print(f"RMSE for p_x: {rmse_x:.4f} m")

        # ---------- 2. p_y --------------------------------------------------------
        y = np.array(data_xyz["/beetle1/uav/cog/odom/pose/pose/position/y"])
        y_ref = (
            np.array(data_xyz_ref["/hand/mocap/pose/pose/position/y"])
            - data_xyz_ref["/hand/mocap/pose/pose/position/y"].iloc[0]
            + y[0]
        )
        _plot_with_error(
            axes[1],
            t,
            y,
            t_ref,
            y_ref,
            "$p_y$ [m]",
            "$\\lvert e_{p_y} \\rvert$ [m]",
            label_size,
            time_start_rotate,
            time_stop_rotate,
        )

        rmse_y = calculate_rmse(t, y, t_ref, y_ref)
        print(f"RMSE for p_y: {rmse_y:.4f} m")

        # ---------- 3. p_z --------------------------------------------------------
        z = np.array(data_xyz["/beetle1/uav/cog/odom/pose/pose/position/z"])
        z_ref = np.array(data_xyz_ref["/hand/mocap/pose/pose/position/z"])
        _plot_with_error(
            axes[2],
            t,
            z,
            t_ref,
            z_ref,
            "$p_z$ [m]",
            "$\\lvert e_{p_z} \\rvert$ [m]",
            label_size,
            time_start_rotate,
            time_stop_rotate,
        )

        rmse_z = calculate_rmse(t, z, t_ref, z_ref)
        print(f"RMSE for p_z: {rmse_z:.4f} m")

        # ---------- 4. q_w --------------------------------------------------------
        qw = np.array(data_qwxyz["/beetle1/uav/cog/odom/pose/pose/orientation/w"])
        qw_ref = -np.array(data_qwxyz_ref["/hand/mocap/pose/pose/orientation/w"])
        t_q = np.array(data_qwxyz["__time"]) - t_bias
        t_qref = np.array(data_qwxyz_ref["__time"]) - t_bias
        _plot_with_error(
            axes[3],
            t_q,
            qw,
            t_qref,
            qw_ref,
            "$q_w$",
            "$\\lvert e_{q_w} \\rvert$",
            label_size,
            time_start_rotate,
            time_stop_rotate,
        )

        rmse_qw = calculate_rmse(t_q, qw, t_qref, qw_ref)
        print(f"RMSE for q_w: {rmse_qw:.4f}")

        # ---------- 5. q_x --------------------------------------------------------
        qx = np.array(data_qwxyz["/beetle1/uav/cog/odom/pose/pose/orientation/x"])
        qx_ref = -np.array(data_qwxyz_ref["/hand/mocap/pose/pose/orientation/x"])
        _plot_with_error(
            axes[4],
            t_q,
            qx,
            t_qref,
            qx_ref,
            "$q_x$",
            "$\\lvert e_{q_x} \\rvert$",
            label_size,
            time_start_rotate,
            time_stop_rotate,
            has_legend=False,
        )

        rmse_qx = calculate_rmse(t_q, qx, t_qref, qx_ref)
        print(f"RMSE for q_x: {rmse_qx:.4f}")

        # ---------- 6. q_y --------------------------------------------------------
        qy = np.array(data_qwxyz["/beetle1/uav/cog/odom/pose/pose/orientation/y"])
        qy_ref = -np.array(data_qwxyz_ref["/hand/mocap/pose/pose/orientation/y"])
        _plot_with_error(
            axes[5],
            t_q,
            qy,
            t_qref,
            qy_ref,
            "$q_y$",
            "$\\lvert e_{q_y} \\rvert$",
            label_size,
            time_start_rotate,
            time_stop_rotate,
        )

        rmse_qy = calculate_rmse(t_q, qy, t_qref, qy_ref)
        print(f"RMSE for q_y: {rmse_qy:.4f}")

        # ---------- 7. q_z --------------------------------------------------------
        qz = np.array(data_qwxyz["/beetle1/uav/cog/odom/pose/pose/orientation/z"])
        qz_ref = -np.array(data_qwxyz_ref["/hand/mocap/pose/pose/orientation/z"])
        _plot_with_error(
            axes[6],
            t_q,
            qz,
            t_qref,
            qz_ref,
            "$q_z$",
            "$\\lvert e_{q_z} \\rvert$",
            label_size,
            time_start_rotate,
            time_stop_rotate,
        )

        rmse_qz = calculate_rmse(t_q, qz, t_qref, qz_ref)
        print(f"RMSE for q_z: {rmse_qz:.4f}")

        # ---------- Shared X-axis -------------------------------------------------
        for ax in axes[:-1]:
            ax.tick_params(labelbottom=False)
        axes[-1].set_xlabel("Time $t$ [s]", fontsize=label_size)

        plt.tight_layout()
        fig.subplots_adjust(hspace=0.25)
        plt.show()

    if plot_type == 1:
        # ====== Data acquisition (same as original) ==============================
        # --- xyz ---
        data_xyz = data[
            [
                "__time",
                "/beetle1/uav/cog/odom/pose/pose/position/x",
                "/beetle1/uav/cog/odom/pose/pose/position/y",
                "/beetle1/uav/cog/odom/pose/pose/position/z",
            ]
        ].dropna()
        data_xyz_ref = data[
            [
                "__time",
                "/hand/mocap/pose/pose/position/x",
                "/hand/mocap/pose/pose/position/y",
                "/hand/mocap/pose/pose/position/z",
            ]
        ].dropna()

        # --- quaternion ---
        data_qwxyz = data[
            [
                "__time",
                "/beetle1/uav/cog/odom/pose/pose/orientation/w",
                "/beetle1/uav/cog/odom/pose/pose/orientation/x",
                "/beetle1/uav/cog/odom/pose/pose/orientation/y",
                "/beetle1/uav/cog/odom/pose/pose/orientation/z",
            ]
        ].dropna()
        data_qwxyz_ref = data[
            [
                "__time",
                "/hand/mocap/pose/pose/orientation/w",
                "/hand/mocap/pose/pose/orientation/x",
                "/hand/mocap/pose/pose/orientation/y",
                "/hand/mocap/pose/pose/orientation/z",
            ]
        ].dropna()

        # convert to euler
        data_euler_ref = pd.DataFrame()
        data_euler_ref["__time"] = data_qwxyz_ref["__time"]
        data_euler_ref["roll"], data_euler_ref["pitch"], data_euler_ref["yaw"] = quat2euler(
            data_qwxyz_ref["/hand/mocap/pose/pose/orientation/w"],
            data_qwxyz_ref["/hand/mocap/pose/pose/orientation/x"],
            data_qwxyz_ref["/hand/mocap/pose/pose/orientation/y"],
            data_qwxyz_ref["/hand/mocap/pose/pose/orientation/z"],
            sequence="ZYX",
            degrees=True,
        )

        # interpolate the real quaternion date
        t_ref = np.array(data_qwxyz_ref["__time"])
        t = np.array(data_qwxyz["__time"])

        data_qwxyz_interp = pd.DataFrame()
        data_qwxyz_interp["__time"] = t_ref
        data_qwxyz_interp["/beetle1/uav/cog/odom/pose/pose/orientation/w"] = np.interp(
            t_ref, t, data_qwxyz["/beetle1/uav/cog/odom/pose/pose/orientation/w"]
        )
        data_qwxyz_interp["/beetle1/uav/cog/odom/pose/pose/orientation/x"] = np.interp(
            t_ref, t, data_qwxyz["/beetle1/uav/cog/odom/pose/pose/orientation/x"]
        )
        data_qwxyz_interp["/beetle1/uav/cog/odom/pose/pose/orientation/y"] = np.interp(
            t_ref, t, data_qwxyz["/beetle1/uav/cog/odom/pose/pose/orientation/y"]
        )
        data_qwxyz_interp["/beetle1/uav/cog/odom/pose/pose/orientation/z"] = np.interp(
            t_ref, t, data_qwxyz["/beetle1/uav/cog/odom/pose/pose/orientation/z"]
        )

        # calculate the quaternion error
        ew, ex, ey, ez = calculate_quat_error(
            data_qwxyz_interp["/beetle1/uav/cog/odom/pose/pose/orientation/w"],
            data_qwxyz_interp["/beetle1/uav/cog/odom/pose/pose/orientation/x"],
            data_qwxyz_interp["/beetle1/uav/cog/odom/pose/pose/orientation/y"],
            data_qwxyz_interp["/beetle1/uav/cog/odom/pose/pose/orientation/z"],
            data_qwxyz_ref["/hand/mocap/pose/pose/orientation/w"],
            data_qwxyz_ref["/hand/mocap/pose/pose/orientation/x"],
            data_qwxyz_ref["/hand/mocap/pose/pose/orientation/y"],
            data_qwxyz_ref["/hand/mocap/pose/pose/orientation/z"],
        )

        e_roll, e_pitch, e_yaw = quat2euler(ew, ex, ey, ez, sequence="ZYX", degrees=True)

        data_euler = pd.DataFrame()
        data_euler["__time"] = t_ref
        data_euler["roll"] = e_roll.to_numpy() + data_euler_ref["roll"].to_numpy()
        data_euler["pitch"] = e_pitch.to_numpy() + data_euler_ref["pitch"].to_numpy()
        data_euler["yaw"] = e_yaw.to_numpy() + data_euler_ref["yaw"].to_numpy()

        # ====== Plot settings =====================================================
        plt.style.use(["science", "grid"])
        plt.rcParams.update({"font.size": 11})
        label_size = 14
        t_bias = max(data_xyz["__time"].iloc[0], data_xyz_ref["__time"].iloc[0])
        time_start_rotate = 28.6993
        time_stop_rotate = 35.2177

        fig, axes = plt.subplots(6, 1, sharex=True, figsize=(7, 6))

        # ---------- 1. p_x --------------------------------------------------------
        t = np.array(data_xyz["__time"]) - t_bias
        x = np.array(data_xyz["/beetle1/uav/cog/odom/pose/pose/position/x"])
        t_ref = np.array(data_xyz_ref["__time"]) - t_bias
        x_ref = (
            np.array(data_xyz_ref["/hand/mocap/pose/pose/position/x"])
            - data_xyz_ref["/hand/mocap/pose/pose/position/x"].iloc[0]
            + x[0]
        )
        _plot_with_error(
            axes[0],
            t,
            x,
            t_ref,
            x_ref,
            "$p_x$ [m]",
            "$\\lvert e_{p_x} \\rvert$ [m]",
            label_size,
            time_start_rotate,
            time_stop_rotate,
            has_legend=True,
        )

        rmse_x = calculate_rmse(t, x, t_ref, x_ref)
        print(f"RMSE for p_x: {rmse_x:.4f} m")

        # ---------- 2. p_y --------------------------------------------------------
        y = np.array(data_xyz["/beetle1/uav/cog/odom/pose/pose/position/y"])
        y_ref = (
            np.array(data_xyz_ref["/hand/mocap/pose/pose/position/y"])
            - data_xyz_ref["/hand/mocap/pose/pose/position/y"].iloc[0]
            + y[0]
        )
        _plot_with_error(
            axes[1],
            t,
            y,
            t_ref,
            y_ref,
            "$p_y$ [m]",
            "$\\lvert e_{p_y} \\rvert$ [m]",
            label_size,
            time_start_rotate,
            time_stop_rotate,
        )

        rmse_y = calculate_rmse(t, y, t_ref, y_ref)
        print(f"RMSE for p_y: {rmse_y:.4f} m")

        # ---------- 3. p_z --------------------------------------------------------
        z = np.array(data_xyz["/beetle1/uav/cog/odom/pose/pose/position/z"])
        z_ref = np.array(data_xyz_ref["/hand/mocap/pose/pose/position/z"])
        _plot_with_error(
            axes[2],
            t,
            z,
            t_ref,
            z_ref,
            "$p_z$ [m]",
            "$\\lvert e_{p_z} \\rvert$ [m]",
            label_size,
            time_start_rotate,
            time_stop_rotate,
        )

        rmse_z = calculate_rmse(t, z, t_ref, z_ref)
        print(f"RMSE for p_z: {rmse_z:.4f} m")

        # ---------- 4. roll --------------------------------------------------------
        roll = unwrap_angle_sequence(np.array(data_euler["roll"]))
        roll_ref = unwrap_angle_sequence(np.array(data_euler_ref["roll"]))
        t_roll = np.array(data_euler["__time"]) - t_bias
        t_roll_ref = np.array(data_euler_ref["__time"]) - t_bias
        _plot_with_error(
            axes[3],
            t_roll,
            roll,
            t_roll_ref,
            roll_ref,
            "Roll [$^\circ$]",
            "$\\lvert e_{\\rm Roll} \\rvert$ [$^\circ$]",
            label_size,
            time_start_rotate,
            time_stop_rotate,
        )

        rmse_roll = calculate_rmse(t_roll, roll, t_roll_ref, roll_ref, is_yaw=True)
        print(f"RMSE for roll: {rmse_roll:.4f} deg")

        # ---------- 5. pitch --------------------------------------------------------
        pitch = unwrap_angle_sequence(np.array(data_euler["pitch"]))
        pitch_ref = unwrap_angle_sequence(np.array(data_euler_ref["pitch"]))
        t_pitch = np.array(data_euler["__time"]) - t_bias
        t_pitch_ref = np.array(data_euler_ref["__time"]) - t_bias
        _plot_with_error(
            axes[4],
            t_pitch,
            pitch,
            t_pitch_ref,
            pitch_ref,
            "Pitch [$^\circ$]",
            "$\\lvert e_{\\rm Pitch} \\rvert$ [$^\circ$]",
            label_size,
            time_start_rotate,
            time_stop_rotate,
        )

        rmse_pitch = calculate_rmse(t_pitch, pitch, t_pitch_ref, pitch_ref, is_yaw=True)
        print(f"RMSE for pitch: {rmse_pitch:.4f} deg")

        # ---------- 6. yaw --------------------------------------------------------
        yaw = unwrap_angle_sequence(np.array(data_euler["yaw"]))
        yaw_ref = unwrap_angle_sequence(np.array(data_euler_ref["yaw"]))
        t_yaw = np.array(data_euler["__time"]) - t_bias
        t_yaw_ref = np.array(data_euler_ref["__time"]) - t_bias
        _plot_with_error(
            axes[5],
            t_yaw,
            yaw,
            t_yaw_ref,
            yaw_ref,
            "Yaw [$^\circ$]",
            "$\\lvert e_{\\rm Yaw} \\rvert$ [$^\circ$]",
            label_size,
            time_start_rotate,
            time_stop_rotate,
            has_legend=False,
        )

        rmse_yaw = calculate_rmse(t_yaw, yaw, t_yaw_ref, yaw_ref, is_yaw=True)
        print(f"RMSE for yaw: {rmse_yaw:.4f} deg")

        # ---------- Shared X-axis -------------------------------------------------
        for ax in axes[:-1]:
            ax.tick_params(labelbottom=False)
        axes[-1].set_xlabel("Time $t$ [s]", fontsize=label_size)

        plt.tight_layout()
        fig.subplots_adjust(hspace=0.25)
        plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Plot the trajectory. Please use plotjuggler to generate the csv file."
    )
    parser.add_argument("file_path", type=str, help="The file name of the trajectory")
    parser.add_argument("--type", type=int, default=0, help="The type of plotting")

    args = parser.parse_args()

    main(args.file_path, args.type)
