'''
 Created by jinjie on 24/05/28.
'''
from tf_conversions import transformations as tf
import matplotlib.pyplot as plt
import numpy as np
import scienceplots

legend_alpha = 0.3


class Visualizer:
    def __init__(self, N_sim, nx, nu, x0):
        self.x_sim_all = np.ndarray((N_sim + 1, nx))
        self.u_sim_all = np.ndarray((N_sim, nu))
        self.x_sim_all[0, :] = x0
        self.comp_time = np.zeros(N_sim)

        self.data_idx = 0

    def update(self, i, x, u):
        self.x_sim_all[i + 1, :] = x
        self.u_sim_all[i, :] = u

        self.data_idx = i + 1

    def visualize(self, ocp_model_name: str, sim_model_name: str, ts_ctrl: float, ts_sim: float, t_total_sim: float,
                  t_servo_ctrl: float = 0.0, t_servo_sim: float = 0.0, t_sqp_start: float = 0, t_sqp_end: float = 0):
        x_sim_all = self.x_sim_all
        u_sim_all = self.u_sim_all

        is_plot_sqp = False
        if t_sqp_start != t_sqp_end and t_sqp_end > t_sqp_start:
            is_plot_sqp = True

        fig = plt.figure(figsize=(20, 15))
        fig.suptitle(
            f"Controller: {ocp_model_name}, ts_ctrl = {ts_ctrl} s, servo delay: {t_servo_ctrl} s\n"
            f"Simulator: {sim_model_name}, ts_sim = {ts_sim} s, servo delay: {t_servo_sim} s"
        )

        time_data_x = np.arange(self.data_idx) * ts_sim

        plt.subplot(5, 2, 1)
        plt.plot(time_data_x, x_sim_all[:self.data_idx, 0], label="x")
        plt.plot(time_data_x, x_sim_all[:self.data_idx, 1], label="y")
        plt.plot(time_data_x, x_sim_all[:self.data_idx, 2], label="z")
        plt.legend(framealpha=legend_alpha)
        plt.xlabel("Time (s)")
        plt.xlim([0, t_total_sim])
        plt.ylabel("Position (m)")
        if is_plot_sqp:
            plt.axvspan(t_sqp_start, t_sqp_end, facecolor="orange", alpha=0.2)
            plt.text(1.5, 0.5, "SQP_RTI", horizontalalignment="center", verticalalignment="center")
            plt.text((t_sqp_start + t_sqp_end) / 2, 0.5, "SQP", horizontalalignment="center",
                     verticalalignment="center")
            plt.text(4.0, 0.5, "SQP_RTI", horizontalalignment="center", verticalalignment="center")
        plt.grid(True)

        plt.subplot(5, 2, 3)
        plt.plot(time_data_x, x_sim_all[:self.data_idx, 3], label="vx")
        plt.plot(time_data_x, x_sim_all[:self.data_idx, 4], label="vy")
        plt.plot(time_data_x, x_sim_all[:self.data_idx, 5], label="vz")
        plt.legend(framealpha=legend_alpha)
        plt.xlabel("Time (s)")
        plt.xlim([0, t_total_sim])
        plt.ylabel("Velocity (m/s)")
        plt.grid(True)
        if is_plot_sqp:
            plt.axvspan(t_sqp_start, t_sqp_end, facecolor="orange", alpha=0.2)

        plt.subplot(5, 2, 5)
        plt.plot(time_data_x, x_sim_all[:self.data_idx, 6], label="qw")
        plt.plot(time_data_x, x_sim_all[:self.data_idx, 7], label="qx")
        plt.plot(time_data_x, x_sim_all[:self.data_idx, 8], label="qy")
        plt.plot(time_data_x, x_sim_all[:self.data_idx, 9], label="qz")
        plt.legend(framealpha=legend_alpha)
        plt.xlabel("Time (s)")
        plt.xlim([0, t_total_sim])
        plt.ylabel("Quaternion")
        plt.grid(True)
        if is_plot_sqp:
            plt.axvspan(t_sqp_start, t_sqp_end, facecolor="orange", alpha=0.2)

        plt.subplot(5, 2, 7)
        # use tf2 to convert x_sim_all[:, 6:10] to euler angle
        euler = np.zeros((x_sim_all.shape[0], 3))
        for i in range(x_sim_all.shape[0]):
            qwxyz = x_sim_all[i, 6:10]
            qxyzw = np.concatenate((qwxyz[1:], qwxyz[:1]))
            euler[i, :] = tf.euler_from_quaternion(qxyzw, axes="sxyz")

        plt.plot(time_data_x, euler[:self.data_idx, 0], label="roll")
        plt.plot(time_data_x, euler[:self.data_idx, 1], label="pitch")
        plt.plot(time_data_x, euler[:self.data_idx, 2], label="yaw")
        plt.legend(framealpha=legend_alpha)
        plt.xlabel("Time (s)")
        plt.xlim([0, t_total_sim])
        plt.ylabel("Euler Angle (rad)")
        plt.grid(True)
        if is_plot_sqp:
            plt.axvspan(t_sqp_start, t_sqp_end, facecolor="orange", alpha=0.2)

        plt.subplot(5, 2, 2)
        plt.plot(time_data_x, x_sim_all[:self.data_idx, 10], label="wx")
        plt.plot(time_data_x, x_sim_all[:self.data_idx, 11], label="wy")
        plt.plot(time_data_x, x_sim_all[:self.data_idx, 12], label="wz")
        plt.legend(framealpha=legend_alpha)
        plt.xlabel("Time (s)")
        plt.xlim([0, t_total_sim])
        plt.ylabel("Body Rate (rad/s)")
        plt.grid(True)
        if is_plot_sqp:
            plt.axvspan(t_sqp_start, t_sqp_end, facecolor="orange", alpha=0.2)

        plt.subplot(5, 2, 4)
        if x_sim_all.shape[1] > 13:
            plt.plot(time_data_x, x_sim_all[:self.data_idx, 13], label="a1")
            plt.plot(time_data_x, x_sim_all[:self.data_idx, 14], label="a2")
            plt.plot(time_data_x, x_sim_all[:self.data_idx, 15], label="a3")
            plt.plot(time_data_x, x_sim_all[:self.data_idx, 16], label="a4")
        plt.legend(framealpha=legend_alpha)
        plt.xlabel("Time (s)")
        plt.xlim([0, t_total_sim])
        plt.ylabel("Servo Angle (rad)")
        plt.grid(True)
        if is_plot_sqp:
            plt.axvspan(t_sqp_start, t_sqp_end, facecolor="orange", alpha=0.2)

        plt.subplot(5, 2, 6)
        plt.plot(time_data_x[1:], u_sim_all[:self.data_idx - 1, 0], label="ft1")
        plt.plot(time_data_x[1:], u_sim_all[:self.data_idx - 1, 1], label="ft2")
        plt.plot(time_data_x[1:], u_sim_all[:self.data_idx - 1, 2], label="ft3")
        plt.plot(time_data_x[1:], u_sim_all[:self.data_idx - 1, 3], label="ft4")
        plt.legend(framealpha=legend_alpha)
        plt.xlabel("Time (s)")
        plt.xlim([0, t_total_sim])
        plt.ylabel("Thrust (N)")
        plt.grid(True)
        if is_plot_sqp:
            plt.axvspan(t_sqp_start, t_sqp_end, facecolor="orange", alpha=0.2)

        plt.subplot(5, 2, 8)
        plt.plot(time_data_x[1:], u_sim_all[:self.data_idx - 1, 4], label="a1c")
        plt.plot(time_data_x[1:], u_sim_all[:self.data_idx - 1, 5], label="a2c")
        plt.plot(time_data_x[1:], u_sim_all[:self.data_idx - 1, 6], label="a3c")
        plt.plot(time_data_x[1:], u_sim_all[:self.data_idx - 1, 7], label="a4c")
        plt.legend(framealpha=legend_alpha)
        plt.xlabel("Time (s)")
        plt.xlim([0, t_total_sim])
        plt.ylabel("Servo Sngle Cmd (rad)")
        plt.grid(True)
        if is_plot_sqp:
            plt.axvspan(t_sqp_start, t_sqp_end, facecolor="orange", alpha=0.2)

        print("Average computation time: ", np.mean(self.comp_time))
        plt.subplot(5, 2, 9)
        plt.plot(time_data_x, self.comp_time)
        plt.xlabel("Time (s)")
        plt.xlim([0, t_total_sim])
        plt.ylabel("Computation Time (s)")
        plt.grid(True)

        plt.tight_layout(rect=[0, 0.03, 1, 0.95])

        plt.show()

    def visualize_less(self, ocp_model_name: str, sim_model_name: str, ts_ctrl: float, ts_sim: float,
                       t_total_sim: float, t_servo_ctrl: float = 0.0, t_servo_sim: float = 0.0):

        plt.style.use(["science", "grid"])

        # set font size
        plt.rcParams.update({'font.size': 11})  # default is 10
        label_size = 12

        # # to avoid the warning of font in pdf check. Seems no need after using scienceplots.
        # matplotlib.rcParams['pdf.fonttype'] = 42
        # matplotlib.rcParams['ps.fonttype'] = 42

        x_sim_all = self.x_sim_all
        u_sim_all = self.u_sim_all

        fig = plt.figure(figsize=(7, 4.5))
        # title = str(f"Ctrl = {ocp_model_name}, ts ctrl = {ts_ctrl} s, servo delay = {t_servo_ctrl} s")
        # title = title.replace("_", r"\_")
        # fig.suptitle(title)

        ax = plt.subplot(211)

        time_data_x = np.arange(self.data_idx) * ts_sim

        plt.plot(time_data_x, x_sim_all[:self.data_idx, 0], label="x")
        plt.plot(time_data_x, x_sim_all[:self.data_idx, 1], label="y")
        plt.plot(time_data_x, x_sim_all[:self.data_idx, 2], label="z")
        plt.ylabel("Position (m)", fontsize=label_size)
        plt.legend(framealpha=legend_alpha, ncol=3, bbox_to_anchor=(0.0, 0.65), loc="lower left")

        euler = np.zeros((x_sim_all.shape[0], 3))
        for i in range(x_sim_all.shape[0]):
            qwxyz = x_sim_all[i, 6:10]
            qxyzw = np.concatenate((qwxyz[1:], qwxyz[:1]))
            euler[i, :] = tf.euler_from_quaternion(qxyzw, axes="sxyz")
        euler = euler * 180 / np.pi

        # plot the y in right axis
        ax_right = ax.twinx()
        ax_right.plot(time_data_x, euler[:self.data_idx, 0], label="roll", linestyle="--")
        ax_right.plot(time_data_x, euler[:self.data_idx, 1], label="pitch", linestyle="--")
        ax_right.plot(time_data_x, euler[:self.data_idx, 2], label="yaw", linestyle="--")
        ax_right.set_ylabel("Euler Angle ($^\\circ$)", fontsize=label_size)

        plt.legend(framealpha=legend_alpha, ncol=3, bbox_to_anchor=(0.45, 0.65), loc="lower left")
        # plt.xlim([0, t_total_sim])

        time_data_u = np.arange(self.data_idx - 1) * ts_sim + ts_sim

        ax2 = plt.subplot(212, sharex=ax)
        plt.plot(time_data_u, u_sim_all[:self.data_idx - 1, 0], label="$f_{c1}$")
        plt.plot(time_data_u, u_sim_all[:self.data_idx - 1, 1], label="$f_{c2}$")
        plt.plot(time_data_u, u_sim_all[:self.data_idx - 1, 2], label="$f_{c3}$")
        plt.plot(time_data_u, u_sim_all[:self.data_idx - 1, 3], label="$f_{c4}$")
        plt.ylabel("Thrust Cmd. (N)", fontsize=label_size)
        plt.xlabel("Time (s)", fontsize=label_size)
        plt.legend(framealpha=legend_alpha, ncol=4, bbox_to_anchor=(0.1, 0.75), loc="lower left")

        ax2_right = ax2.twinx()
        ax2_right.plot(time_data_u, u_sim_all[:self.data_idx - 1, 4] * 180 / np.pi, label="$\\alpha_{c1}$",
                       linestyle="--")
        ax2_right.plot(time_data_u, u_sim_all[:self.data_idx - 1, 5] * 180 / np.pi, label="$\\alpha_{c2}$",
                       linestyle="--")
        ax2_right.plot(time_data_u, u_sim_all[:self.data_idx - 1, 6] * 180 / np.pi, label="$\\alpha_{c3}$",
                       linestyle="--")
        ax2_right.plot(time_data_u, u_sim_all[:self.data_idx - 1, 7] * 180 / np.pi, label="$\\alpha_{c4}$",
                       linestyle="--")
        ax2_right.set_ylabel("Servo Angle Cmd. ($^\\circ$)", fontsize=label_size)
        plt.legend(framealpha=legend_alpha, ncol=4, bbox_to_anchor=(0.1, 0.00), loc="lower left")

        plt.xlim([-0.1, t_total_sim])

        # plt.ylabel("Servo Angle Cmd (rad)", fontsize=label_size)
        # plt.ylim([-1.0, 1.0])  # -0.8, 0.8; -1.6, 1.6

        # plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        plt.tight_layout()
        fig.subplots_adjust(hspace=0.15)

        plt.show()

    def visualize_rpy(self, ocp_model_name: str, sim_model_name: str, ts_ctrl: float, ts_sim: float,
                      t_total_sim: float, t_servo_ctrl: float = 0.0, t_servo_sim: float = 0.0):

        plt.style.use(["science", "grid"])

        # set font size
        plt.rcParams.update({'font.size': 11})  # default is 10
        label_size = 14

        x_sim_all = self.x_sim_all
        u_sim_all = self.u_sim_all

        fig = plt.figure(figsize=(3.5, 2.0))
        title = str(f"{ocp_model_name}")
        title = title.replace("_", r"\_")
        # fig.title(title)

        time_data_x = np.arange(self.data_idx) * ts_sim

        euler = np.zeros((x_sim_all.shape[0], 3))
        for i in range(x_sim_all.shape[0]):
            qwxyz = x_sim_all[i, 6:10]
            qxyzw = np.concatenate((qwxyz[1:], qwxyz[:1]))
            euler[i, :] = tf.euler_from_quaternion(qxyzw, axes="sxyz")

        # plt.title(title)
        # plot reference as 0.5
        plt.plot([0, t_total_sim], [0.5, 0.5], label="ref", linestyle="-.")
        plt.plot(time_data_x, euler[:self.data_idx, 0], label="roll")
        plt.plot(time_data_x, euler[:self.data_idx, 1], label="pitch")
        plt.plot(time_data_x, euler[:self.data_idx, 2], label="yaw")
        plt.legend(framealpha=legend_alpha, loc="lower right")
        plt.xlabel("Time (s)", fontsize=label_size)
        plt.xlim([0, t_total_sim])
        plt.ylim([-0.02, 0.52])
        plt.ylabel("Euler Angle (rad)", fontsize=label_size)

        # plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        plt.tight_layout()

        plt.show()
