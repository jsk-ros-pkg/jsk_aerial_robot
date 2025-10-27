import pandas as pd
import numpy as np
import scienceplots
import matplotlib.pyplot as plt
import argparse
from scipy.spatial.transform import Rotation as R

legend_alpha = 0.5


def main(file_path, plot_type):
    # Load the data from the csv file
    fly_data = pd.read_csv(file_path)

    # ======= data selection =========
    data_sen_wrench = fly_data[
        [
            "__time",
            "/cfs/data/wrench/force/x",
            "/cfs/data/wrench/force/y",
            "/cfs/data/wrench/force/z",
            "/cfs/data/wrench/torque/x",
            "/cfs/data/wrench/torque/y",
            "/cfs/data/wrench/torque/z",
        ]
    ]
    data_sen_wrench = data_sen_wrench.dropna()
    data_sen_wrench.columns = ["t", "fx", "fy", "fz", "tx", "ty", "tz"]

    data_est_wrench = fly_data[
        [
            "__time",
            "/beetle1/disturbance_wrench/wrench/force/x",
            "/beetle1/disturbance_wrench/wrench/force/y",
            "/beetle1/disturbance_wrench/wrench/force/z",
            "/beetle1/disturbance_wrench/wrench/torque/x",
            "/beetle1/disturbance_wrench/wrench/torque/y",
            "/beetle1/disturbance_wrench/wrench/torque/z",
        ]
    ]
    data_est_wrench = data_est_wrench.dropna()
    data_est_wrench.columns = ["t", "fx", "fy", "fz", "tx", "ty", "tz"]

    # offset: according to the hovering data, offset data are fx = 0.2616N, fy = 0.1609N, fz = -2.1902N,
    # tx = -0.1189Nm, ty = 0.1301Nm, tz = 0.1432Nm
    fly_file_name = file_path.split("/")[-1]
    offset = np.zeros(7)
    if "20250123" in fly_file_name:
        offset[1:] = np.array([0.2616, 0.1609, -2.1902, -0.1189, 0.1301, 0.1432])
    data_est_wrench = data_est_wrench - offset

    data_imu = fly_data[
        [
            "__time",
            "/beetle1/imu/acc_data[0]",
            "/beetle1/imu/acc_data[1]",
            "/beetle1/imu/acc_data[2]",
            "/beetle1/imu/gyro_data[0]",
            "/beetle1/imu/gyro_data[1]",
            "/beetle1/imu/gyro_data[2]",
        ]
    ]
    data_imu = data_imu.dropna()
    data_imu.columns = ["t", "ax", "ay", "az", "wx", "wy", "wz"]

    # # ======= orientation =========
    data_qwxyz = fly_data[
        [
            "__time",
            "/beetle1/uav/cog/odom/pose/pose/orientation/w",
            "/beetle1/uav/cog/odom/pose/pose/orientation/x",
            "/beetle1/uav/cog/odom/pose/pose/orientation/y",
            "/beetle1/uav/cog/odom/pose/pose/orientation/z",
        ]
    ]
    data_qwxyz = data_qwxyz.dropna()
    data_qwxyz.columns = ["__time", "qw", "qx", "qy", "qz"]

    # ======= preprocessing =========
    t_ref = np.array(data_sen_wrench["t"]) - data_sen_wrench["t"].iloc[0]
    time_duration = data_sen_wrench["t"].iloc[-1] - data_sen_wrench["t"].iloc[0]
    # print(data_sen_wrench.shape)  # (5260, 7)

    t_real_start = 0  # s
    t_real_end = t_real_start + time_duration
    data_est_wrench_sel = data_est_wrench[
        (data_est_wrench["t"] >= t_real_start + data_est_wrench["t"].iloc[0])
        & (data_est_wrench["t"] <= data_est_wrench["t"].iloc[0] + t_real_end)
    ]
    t_real = np.array(data_est_wrench_sel["t"]) - data_est_wrench_sel["t"].iloc[0]

    data_imu_sel = data_imu[
        (data_imu["t"] >= t_real_start + data_imu["t"].iloc[0]) & (data_imu["t"] <= data_imu["t"].iloc[0] + t_real_end)
    ]
    t_imu = np.array(data_imu_sel["t"]) - data_imu_sel["t"].iloc[0]

    data_qwxyz_sel = data_qwxyz[
        (data_qwxyz["__time"] >= t_real_start + data_qwxyz["__time"].iloc[0])
        & (data_qwxyz["__time"] <= data_qwxyz["__time"].iloc[0] + t_real_end)
    ]
    # print(data_qwxyz_sel.shape)  # (5865, 5)

    # for all data in data_sen_wrench, use data_qwxyz_sel to do coordinate transform. create a new data_sen_wrench_body
    data_sen_wrench_body = data_sen_wrench.copy()
    qx = data_qwxyz_sel["qx"].to_numpy()
    qy = data_qwxyz_sel["qy"].to_numpy()
    qz = data_qwxyz_sel["qz"].to_numpy()
    qw = data_qwxyz_sel["qw"].to_numpy()
    for i in range(len(data_sen_wrench)):
        rot = R.from_quat([qx[i], qy[i], qz[i], qw[i]])
        # transform wrench from world frame to body frame
        force_body = rot.inv().apply(
            [data_sen_wrench["fx"].iloc[i], data_sen_wrench["fy"].iloc[i], data_sen_wrench["fz"].iloc[i]]
        )
        torque_body = rot.inv().apply(
            [data_sen_wrench["tx"].iloc[i], data_sen_wrench["ty"].iloc[i], data_sen_wrench["tz"].iloc[i]]
        )

        # make the final coordinate transform. This is the conversion from sensor frame to world frame, but
        # I make it here by my observation. TODO: the torque seems to be correct, but the force is not. I need to check.
        data_sen_wrench_body["fx"].iloc[i] = force_body[0]
        data_sen_wrench_body["fy"].iloc[i] = force_body[1]
        data_sen_wrench_body["fz"].iloc[i] = force_body[2]
        data_sen_wrench_body["tx"].iloc[i] = torque_body[0]
        data_sen_wrench_body["ty"].iloc[i] = torque_body[1]
        data_sen_wrench_body["tz"].iloc[i] = torque_body[2]

    # filter data_sen_wrench_body to get data_est_wrench_body_filtered
    data_est_wrench_body_filtered = data_sen_wrench_body.copy()
    data_est_wrench_body_filtered["fx"] = data_est_wrench_body_filtered["fx"].rolling(window=10).mean()
    data_est_wrench_body_filtered["fy"] = data_est_wrench_body_filtered["fy"].rolling(window=10).mean()
    data_est_wrench_body_filtered["fz"] = data_est_wrench_body_filtered["fz"].rolling(window=10).mean()
    data_est_wrench_body_filtered["tx"] = data_est_wrench_body_filtered["tx"].rolling(window=10).mean()
    data_est_wrench_body_filtered["ty"] = data_est_wrench_body_filtered["ty"].rolling(window=10).mean()
    data_est_wrench_body_filtered["tz"] = data_est_wrench_body_filtered["tz"].rolling(window=10).mean()

    # ======= plotting =========
    if plot_type == 0:
        plt.style.use(["science", "grid"])

        plt.rcParams.update({"font.size": 11})  # default is 10
        label_size = 14

        fig, axes = plt.subplots(4, 2, sharex=True, figsize=(7, 7))
        axes = axes.flatten()  # makes it easy to index 0–5

        color_ref = "#0C5DA5"
        color_real = "#FF2C00"

        # --------------- Wrench -----------------
        keys = ["fx", "tx", "fy", "ty", "fz", "tz"]
        ylabels = [
            r"${}^B\hat{f}_{de,x}$ [N]",
            r"${}^B\hat{\tau}_{de,x}$ [N$\cdot$m]",
            r"${}^B\hat{f}_{de,y}$ [N]",
            r"${}^B\hat{\tau}_{de,y}$ [N$\cdot$m]",
            r"${}^B\hat{f}_{de,z}$ [N]",
            r"${}^B\hat{\tau}_{de,z}$ [N$\cdot$m]",
        ]

        for i, (key, ylabel) in enumerate(zip(keys, ylabels)):
            ax = axes[i]
            # plot ref and real
            ax.plot(
                t_ref, data_sen_wrench_body[key], linestyle="--", label="ground truth (gt)", color=color_ref, alpha=0.3
            )
            ax.plot(t_ref, data_est_wrench_body_filtered[key], linestyle="-", label="gt filtered", color=color_ref)

            ax.plot(t_real, data_est_wrench_sel[key], linestyle="-", label="estimation", color=color_real)

            # only the first subplot gets a legend
            if i == 4:
                ax.legend(framealpha=legend_alpha, loc="lower left")

            ax.set_ylabel(ylabel, fontsize=label_size)

            ax.axvspan(26, 29, facecolor="#EDB120", alpha=0.3)

            # # bottom‐row plots (i = 4,5) get the shared x‐label
            # if i >= 4:
            #     ax.set_xlabel('Time [s]', fontsize=label_size)
        # ---------------- Imu ----------------
        gravity_const = 9.798  # m/s^2 Tokyo  # TODO: need to do coordinate transform

        ax = axes[6]
        ax.plot(t_imu, data_imu_sel["ax"], linestyle="-", label="ax")
        ax.plot(t_imu, data_imu_sel["ay"], linestyle="-.", label="ay")
        ax.plot(t_imu, data_imu_sel["az"], linestyle=":", label="az")
        ax.legend(framealpha=legend_alpha, loc="center", ncol=3)
        ax.set_ylabel("$^B\hat{a}$ [m/s$^2$]", fontsize=label_size)
        ax.set_xlabel("Time $t$ [s]", fontsize=label_size)
        # --------------- Gyro -----------------
        ax = axes[7]
        ax.plot(t_imu, data_imu_sel["wx"], linestyle="-", label="$\omega_x$")
        ax.plot(t_imu, data_imu_sel["wy"], linestyle="-.", label="$\omega_y$")
        ax.plot(t_imu, data_imu_sel["wz"], linestyle=":", label="$\omega_z$")
        ax.legend(framealpha=legend_alpha, loc="upper center", ncol=3)
        ax.set_ylabel("$^B\hat{\omega}$ [rad/s]", fontsize=label_size)
        ax.set_xlabel("Time $t$ [s]", fontsize=label_size)

        # --------------------------------
        plt.tight_layout()

        plt.show()

    else:
        print("Invalid type")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Plot the estimated wrench with true value. Please use plotjuggler to generate the csv file."
    )
    parser.add_argument("file_path", type=str, help="The file name of the trajectory")
    parser.add_argument("--type", type=int, default=0, help="The type of the trajectory")

    args = parser.parse_args()

    main(args.file_path, args.type)
