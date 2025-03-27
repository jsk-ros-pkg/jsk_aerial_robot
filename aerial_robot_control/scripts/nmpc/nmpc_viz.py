from tf_conversions import transformations as tf
import matplotlib.pyplot as plt
from math import ceil
import numpy as np
import scienceplots     # DON'T DELETE, it is used indirectly in matplotlib for fonts

legend_alpha = 0.3


class Visualizer:
    def __init__(
            self,
            robot_arch,
            N_sim,
            nx,
            nu,
            x0,
            tilt = False,
            include_servo_model = False,
            include_thrust_model = False,
            include_cog_dist_model = False,
            include_cog_dist_est = False,
            is_record_diff_u=False
        ):
        # Store robot architecture
        self.is_bi, self.is_tri, self.is_qd = False, False, False
        if robot_arch == 'bi':
            self.is_bi = True
        elif robot_arch == 'tri':
            self.is_tri = True
        elif robot_arch == 'qd':
            self.is_qd = True
        else:
            raise ValueError("This robot architecture is not implemented yet!")

        # Store model properties
        self.tilt = tilt
        self.include_servo_model = include_servo_model
        self.include_thrust_model = include_thrust_model
        self.include_cog_dist_model = include_cog_dist_model
        self.include_cog_dist_est = include_cog_dist_est
        self.is_record_diff_u = is_record_diff_u

        self.x_sim_all = np.ndarray((N_sim + 1, nx))
        self.u_sim_all = np.ndarray((N_sim, nu))

        self.x_sim_all[0, :] = x0
        self.comp_time = np.zeros(N_sim)

        if self.is_record_diff_u:
            self.u_sim_mpc_all = np.ndarray((N_sim, nu))

        self.est_disturb_f_w_all = np.ndarray((N_sim, 3))
        self.est_disturb_tau_g_all = np.ndarray((N_sim, 3))

        self.data_idx = 0

    def update(self, i, x, u):
        self.x_sim_all[i + 1, :] = x    # Next time step
        self.u_sim_all[i, :] = u        # Current time step

        self.data_idx = i + 1

    def update_u_mpc(self, i, u_mpc):
        self.u_sim_mpc_all[i, :] = u_mpc

    def update_est_disturb(self, i, est_disturb_f_w, est_disturb_tau_g):
        self.est_disturb_f_w_all[i, :] = est_disturb_f_w
        self.est_disturb_tau_g_all[i, :] = est_disturb_tau_g

    def visualize(
            self,
            ocp_model_name: str,
            sim_model_name: str,
            ts_ctrl: float,
            ts_sim: float,
            t_total_sim: float,
            t_servo_ctrl: float = 0.0,
            t_servo_sim: float = 0.0,
            t_sqp_start: float = 0.0,
            t_sqp_end: float = 0.0
        ):
        x_sim_all = self.x_sim_all
        u_sim_all = self.u_sim_all

        is_plot_sqp = False
        if t_sqp_start != t_sqp_end and t_sqp_end > t_sqp_start:
            is_plot_sqp = True

        # Set up plot
        fig = plt.figure(figsize=(20, 15))
        fig.suptitle(
            f"Controller: {ocp_model_name}, ts_ctrl = {ts_ctrl} s, servo delay: {t_servo_ctrl} s\n"
            f"Simulator: {sim_model_name}, ts_sim = {ts_sim} s, servo delay: {t_servo_sim} s"
        )
        # Number of subplots
        n_plots = 7                                         # States, Controls and Computation Time
        if self.include_servo_model: n_plots += 1           # Additional State: Servo Angle
        if self.include_thrust_model: n_plots += 1          # Additional State: Thrust
        if self.tilt: n_plots += 1                          # Additional Control: Servon Angle
        if self.include_cog_dist_model: n_plots += 2        # Additional State: Disturbance Force and Torque
        if self.include_cog_dist_est: n_plots += 2          # Disturbance Force and Torque
        if hasattr(self, 'u_sim_mpc_all'): n_plots += 2     # MPC controls if needed

        # Timeseries
        time_data_x = np.arange(self.data_idx) * ts_sim

        # Plot Position
        plt.subplot(ceil(n_plots/2), 2, 1)
        plt.plot(time_data_x, x_sim_all[:self.data_idx, 0], label="x")
        plt.plot(time_data_x, x_sim_all[:self.data_idx, 1], label="y")
        plt.plot(time_data_x, x_sim_all[:self.data_idx, 2], label="z")
        plt.legend(framealpha=legend_alpha)
        # plt.xlabel("Time (s)")
        plt.xlim([0, t_total_sim])
        plt.ylabel("Position (m)")
        if is_plot_sqp:
            plt.axvspan(t_sqp_start, t_sqp_end, facecolor="orange", alpha=0.2)
            plt.text(1.5, 0.5, "SQP_RTI", horizontalalignment="center", verticalalignment="center")
            plt.text((t_sqp_start + t_sqp_end) / 2, 0.5, "SQP", horizontalalignment="center",
                     verticalalignment="center")
            plt.text(4.0, 0.5, "SQP_RTI", horizontalalignment="center", verticalalignment="center")
        plt.grid(True)

        # Plot Velocity
        plt.subplot(ceil(n_plots/2), 2, 3)
        plt.plot(time_data_x, x_sim_all[:self.data_idx, 3], label="vx")
        plt.plot(time_data_x, x_sim_all[:self.data_idx, 4], label="vy")
        plt.plot(time_data_x, x_sim_all[:self.data_idx, 5], label="vz")
        plt.legend(framealpha=legend_alpha)
        # plt.xlabel("Time (s)")
        plt.xlim([0, t_total_sim])
        plt.ylabel("Velocity (m/s)")
        plt.grid(True)
        if is_plot_sqp:
            plt.axvspan(t_sqp_start, t_sqp_end, facecolor="orange", alpha=0.2)

        # Plot Quaternions
        plt.subplot(ceil(n_plots/2), 2, 5)
        plt.plot(time_data_x, x_sim_all[:self.data_idx, 6], label="qw")
        plt.plot(time_data_x, x_sim_all[:self.data_idx, 7], label="qx")
        plt.plot(time_data_x, x_sim_all[:self.data_idx, 8], label="qy")
        plt.plot(time_data_x, x_sim_all[:self.data_idx, 9], label="qz")
        plt.legend(framealpha=legend_alpha)
        # plt.xlabel("Time (s)")
        plt.xlim([0, t_total_sim])
        plt.ylabel("Quaternion")
        plt.grid(True)
        if is_plot_sqp:
            plt.axvspan(t_sqp_start, t_sqp_end, facecolor="orange", alpha=0.2)

        # Use tf2 to convert x_sim_all[:, 6:10] to euler angle
        euler = np.zeros((x_sim_all.shape[0], 3))
        for i in range(x_sim_all.shape[0]):
            qwxyz = x_sim_all[i, 6:10]
            qxyzw = np.concatenate((qwxyz[1:], qwxyz[:1]))
            euler[i, :] = tf.euler_from_quaternion(qxyzw, axes="sxyz")

        # Plot Euler Angles
        plt.subplot(ceil(n_plots/2), 2, 2)
        plt.plot(time_data_x, euler[:self.data_idx, 0], label="roll")
        plt.plot(time_data_x, euler[:self.data_idx, 1], label="pitch")
        plt.plot(time_data_x, euler[:self.data_idx, 2], label="yaw")
        plt.legend(framealpha=legend_alpha)
        # plt.xlabel("Time (s)")
        plt.xlim([0, t_total_sim])
        plt.ylabel("Euler Angle (rad)")
        plt.grid(True)
        if is_plot_sqp:
            plt.axvspan(t_sqp_start, t_sqp_end, facecolor="orange", alpha=0.2)

        # Plot Angular Velocity
        plt.subplot(ceil(n_plots/2), 2, 4)
        plt.plot(time_data_x, x_sim_all[:self.data_idx, 10], label="wx")
        plt.plot(time_data_x, x_sim_all[:self.data_idx, 11], label="wy")
        plt.plot(time_data_x, x_sim_all[:self.data_idx, 12], label="wz")
        plt.legend(framealpha=legend_alpha)
        # plt.xlabel("Time (s)")
        plt.xlim([0, t_total_sim])
        plt.ylabel("Angular Velocity (rad/s)")
        plt.grid(True)
        if is_plot_sqp:
            plt.axvspan(t_sqp_start, t_sqp_end, facecolor="orange", alpha=0.2)

        # Plot Computation Time
        print("Average computation time: ", np.mean(self.comp_time))
        plt.subplot(ceil(n_plots/2), 2, 6)
        plt.plot(time_data_x, self.comp_time)
        plt.xlabel("Time (s)")
        plt.xlim([0, t_total_sim])
        plt.ylabel("Computation Time (s)")
        plt.grid(True)

        # Plot Servo Angle as State
        x_idx = 12
        if self.tilt and self.include_servo_model:
            plt.subplot(ceil(n_plots/2), 2, 7)
            plt.plot(time_data_x, x_sim_all[:self.data_idx, x_idx+1], label="a1s")
            plt.plot(time_data_x, x_sim_all[:self.data_idx, x_idx+2], label="a2s")
            x_idx += 2
            if self.is_tri or self.is_qd: 
                plt.plot(time_data_x, x_sim_all[:self.data_idx, x_idx+1], label="a3s")
                x_idx += 1
            if self.is_qd: 
                plt.plot(time_data_x, x_sim_all[:self.data_idx, x_idx+1], label="a4s")
                x_idx += 1
            plt.legend(framealpha=legend_alpha)
            # plt.xlabel("Time (s)")
            plt.xlim([0, t_total_sim])
            plt.ylabel("Servo Angle State (rad)")
            plt.grid(True)
            if is_plot_sqp:
                plt.axvspan(t_sqp_start, t_sqp_end, facecolor="orange", alpha=0.2)

        # Plot Thrust as State
        if self.include_thrust_model:
            plt.subplot(ceil(n_plots/2), 2, 8)
            plt.plot(time_data_x, x_sim_all[:self.data_idx, x_idx+1], label="ft1s")
            plt.plot(time_data_x, x_sim_all[:self.data_idx, x_idx+2], label="ft2s")
            x_idx += 2
            if self.is_tri or self.is_qd:
                plt.plot(time_data_x, x_sim_all[:self.data_idx, x_idx+1], label="ft3s")
                x_idx += 1
            if self.is_qd:
                plt.plot(time_data_x, x_sim_all[:self.data_idx, x_idx+1], label="ft4s")
                x_idx += 1
            plt.legend(framealpha=legend_alpha)
            # plt.xlabel("Time (s)")
            plt.xlim([0, t_total_sim])
            plt.ylabel("Thrust State (N)")
            plt.grid(True)
            if is_plot_sqp:
                plt.axvspan(t_sqp_start, t_sqp_end, facecolor="orange", alpha=0.2)

        # Plot Thrust as Control Input
        if self.include_thrust_model:
            plt.subplot(ceil(n_plots/2), 2, 10)
        else:
            plt.subplot(ceil(n_plots/2), 2, 8)
        plt.plot(time_data_x[1:], u_sim_all[:self.data_idx - 1, 0], label="ft1c")
        plt.plot(time_data_x[1:], u_sim_all[:self.data_idx - 1, 1], label="ft2c")
        u_idx = 1
        if self.is_tri or self.is_qd:
            plt.plot(time_data_x[1:], u_sim_all[:self.data_idx - 1, 2], label="ft3c")
            u_idx = 2
        if self.is_qd:
            plt.plot(time_data_x[1:], u_sim_all[:self.data_idx - 1, 3], label="ft4c")
            u_idx = 3
        plt.legend(framealpha=legend_alpha)
        # plt.xlabel("Time (s)")
        plt.xlim([0, t_total_sim])
        plt.ylabel("Thrust Cmd (N)")
        plt.grid(True)
        if is_plot_sqp:
            plt.axvspan(t_sqp_start, t_sqp_end, facecolor="orange", alpha=0.2)

        # Plot Servo Angle as Control Input
        if self.tilt:
            if self.include_servo_model:
                plt.subplot(ceil(n_plots/2), 2, 9)
            else:
                plt.subplot(ceil(n_plots/2), 2, 7)
            plt.plot(time_data_x[1:], u_sim_all[:self.data_idx - 1, u_idx+1], label="a1c")
            plt.plot(time_data_x[1:], u_sim_all[:self.data_idx - 1, u_idx+2], label="a2c")
            if self.is_tri or self.is_qd:
                plt.plot(time_data_x[1:], u_sim_all[:self.data_idx - 1, u_idx+3], label="a3c")
            if self.is_qd:
                plt.plot(time_data_x[1:], u_sim_all[:self.data_idx - 1, u_idx+4], label="a4c")
            plt.legend(framealpha=legend_alpha)
            # plt.xlabel("Time (s)")
            plt.xlim([0, t_total_sim])
            plt.ylabel("Servo Angle Cmd (rad)")
            plt.grid(True)
            if is_plot_sqp:
                plt.axvspan(t_sqp_start, t_sqp_end, facecolor="orange", alpha=0.2)

        plot_idx = 10
        # Plot seperate MPC control variable
        if self.is_record_diff_u:
            plt.subplot(ceil(n_plots/2), 2, 11)
            plt.plot(time_data_x[1:], self.u_sim_mpc_all[:self.data_idx - 1, 4], label="a1c_mpc")
            plt.plot(time_data_x[1:], self.u_sim_mpc_all[:self.data_idx - 1, 5], label="a2c_mpc")
            if self.is_tri or self.is_qd:
                plt.plot(time_data_x[1:], self.u_sim_mpc_all[:self.data_idx - 1, 6], label="a3c_mpc")
            if self.is_qd:
                plt.plot(time_data_x[1:], self.u_sim_mpc_all[:self.data_idx - 1, 7], label="a4c_mpc")
            plt.legend(framealpha=legend_alpha, loc="upper left")
            plt.xlim([0, t_total_sim])
            plt.ylabel("Servo Angle Cmd MPC (rad)")
            plt.grid(True)

            plt.subplot(ceil(n_plots/2), 2, 12)
            plt.plot(time_data_x[1:], self.u_sim_mpc_all[:self.data_idx - 1, 0], label="ft1c_mpc")
            plt.plot(time_data_x[1:], self.u_sim_mpc_all[:self.data_idx - 1, 1], label="ft2c_mpc")
            if self.is_tri or self.is_qd:
                plt.plot(time_data_x[1:], self.u_sim_mpc_all[:self.data_idx - 1, 2], label="ft3c_mpc")
            if self.is_qd:
                plt.plot(time_data_x[1:], self.u_sim_mpc_all[:self.data_idx - 1, 3], label="ft4c_mpc")
            plt.legend(framealpha=legend_alpha, loc="upper left")
            plt.xlim([0, t_total_sim])
            plt.ylabel("Thrust Cmd MPC (N)")
            plt.grid(True)

            plot_idx += 2

        if self.include_cog_dist_model:
            # Plot Disturbance Force as State in NMPC
            plt.subplot(ceil(n_plots/2), 2, plot_idx+1)
            plt.plot(time_data_x, x_sim_all[:self.data_idx, x_idx+1], label="f_d_x")
            plt.plot(time_data_x, x_sim_all[:self.data_idx, x_idx+2], label="f_d_y")
            plt.plot(time_data_x, x_sim_all[:self.data_idx, x_idx+3], label="f_d_z")
            plt.legend(framealpha=legend_alpha, loc="upper left")
            plt.xlim([0, t_total_sim])
            plt.ylabel("Disturbance Force (N)")
            plt.grid(True)

            # Plot Disturbance Torque as State in NMPC
            plt.subplot(ceil(n_plots/2), 2, plot_idx+2)
            plt.plot(time_data_x, x_sim_all[:self.data_idx, x_idx+4], label="tau_d_x")
            plt.plot(time_data_x, x_sim_all[:self.data_idx, x_idx+5], label="tau_d_y")
            plt.plot(time_data_x, x_sim_all[:self.data_idx, x_idx+6], label="tau_d_z")
            plt.legend(framealpha=legend_alpha, loc="upper left")
            plt.xlim([0, t_total_sim])
            plt.ylabel("Disturbance Torque (N*m)")
            plt.grid(True)

            plot_idx += 2

        if self.include_cog_dist_est:
            # Plot estimated Disturbance Force by MHE
            plt.subplot(ceil(n_plots/2), 2, plot_idx+1)
            plt.plot(time_data_x, self.est_disturb_f_w_all[:self.data_idx, 0], label="est. f_d_x")
            plt.plot(time_data_x, self.est_disturb_f_w_all[:self.data_idx, 1], label="est. f_d_y")
            plt.plot(time_data_x, self.est_disturb_f_w_all[:self.data_idx, 2], label="est. f_d_z")
            plt.legend(framealpha=legend_alpha, loc="upper left")
            plt.xlim([0, t_total_sim])
            plt.ylabel("Est. Disturbance Force (N)")
            plt.grid(True)

            # Plot estimated Disturbance Torque by MHE
            plt.subplot(ceil(n_plots/2), 2, plot_idx+2)
            plt.plot(time_data_x, self.est_disturb_tau_g_all[:self.data_idx, 0], label="est. tau_d_x")
            plt.plot(time_data_x, self.est_disturb_tau_g_all[:self.data_idx, 1], label="est. tau_d_y")
            plt.plot(time_data_x, self.est_disturb_tau_g_all[:self.data_idx, 2], label="est. tau_d_z")
            plt.legend(framealpha=legend_alpha, loc="upper left")
            plt.xlim([0, t_total_sim])
            plt.ylabel("Est. Disturbance Torque (N*m)")
            plt.grid(True)

        plt.tight_layout(rect=[0, 0.03, 1, 0.95])

        plt.show()

    def visualize_less(self, ts_sim: float, t_total_sim: float):

        plt.style.use(["science", "grid"])

        # Font size
        plt.rcParams.update({'font.size': 11})      # Default is 10
        label_size = 12

        # To avoid warning of font in pdf check -> Seems no need after using scienceplots.
        # matplotlib.rcParams['pdf.fonttype'] = 42
        # matplotlib.rcParams['ps.fonttype'] = 42

        x_sim_all = self.x_sim_all
        u_sim_all = self.u_sim_all

        # Timeseries
        time_data_x = np.arange(self.data_idx) * ts_sim
        time_data_u = np.arange(self.data_idx - 1) * ts_sim + ts_sim

        # Set up plot
        fig = plt.figure(figsize=(7, 4.5))
        # title = str(f"Ctrl = {ocp_model_name}, ts ctrl = {ts_ctrl} s, servo delay = {t_servo_ctrl} s")
        # title = title.replace("_", r"\_")
        # fig.suptitle(title)
        ax = plt.subplot(211)

        # Plot Position
        plt.plot(time_data_x, x_sim_all[:self.data_idx, 0], label="x")
        plt.plot(time_data_x, x_sim_all[:self.data_idx, 1], label="y")
        plt.plot(time_data_x, x_sim_all[:self.data_idx, 2], label="z")
        plt.ylabel("Position (m)", fontsize=label_size)
        plt.legend(framealpha=legend_alpha, ncol=3, bbox_to_anchor=(0.0, 0.65), loc="lower left")

        # Convert Quaternions into Euler Angles using tf
        euler = np.zeros((x_sim_all.shape[0], 3))
        for i in range(x_sim_all.shape[0]):
            qwxyz = x_sim_all[i, 6:10]
            qxyzw = np.concatenate((qwxyz[1:], qwxyz[:1]))
            euler[i, :] = tf.euler_from_quaternion(qxyzw, axes="sxyz")
        euler = euler * 180 / np.pi     # in degree

        # Plot Euler Angles (with y-axis on the right side)
        ax_right = ax.twinx()
        ax_right.plot(time_data_x, euler[:self.data_idx, 0], label="roll", linestyle="--")
        ax_right.plot(time_data_x, euler[:self.data_idx, 1], label="pitch", linestyle="--")
        ax_right.plot(time_data_x, euler[:self.data_idx, 2], label="yaw", linestyle="--")
        ax_right.set_ylabel("Euler Angle ($^\\circ$)", fontsize=label_size)
        plt.legend(framealpha=legend_alpha, ncol=3, bbox_to_anchor=(0.45, 0.7), loc="lower left")

        # Plot Thrust as Control Input
        ax2 = plt.subplot(212, sharex=ax)
        plt.plot(time_data_u, u_sim_all[:self.data_idx - 1, 0], label="$f_{c1}$")
        plt.plot(time_data_u, u_sim_all[:self.data_idx - 1, 1], label="$f_{c2}$")
        if self.is_tri or self.is_qd:
            plt.plot(time_data_u, u_sim_all[:self.data_idx - 1, 2], label="$f_{c3}$")
        if self.is_qd:
            plt.plot(time_data_u, u_sim_all[:self.data_idx - 1, 3], label="$f_{c4}$")
        plt.ylabel("Thrust Cmd. (N)", fontsize=label_size)
        plt.xlabel("Time (s)", fontsize=label_size)
        plt.legend(framealpha=legend_alpha, ncol=4, bbox_to_anchor=(0.1, 0.75), loc="lower left")

        # Plot Sensor Angle as Control Input (in degree)
        if self.tilt:
            ax2_right = ax2.twinx()
            ax2_right.plot(time_data_u, u_sim_all[:self.data_idx - 1, 4] * 180 / np.pi, label="$\\alpha_{c1}$",
                        linestyle="--")
            ax2_right.plot(time_data_u, u_sim_all[:self.data_idx - 1, 5] * 180 / np.pi, label="$\\alpha_{c2}$",
                        linestyle="--")
            if self.is_tri or self.is_qd:
                ax2_right.plot(time_data_u, u_sim_all[:self.data_idx - 1, 6] * 180 / np.pi, label="$\\alpha_{c3}$",
                        linestyle="--")
            if self.is_qd:
                ax2_right.plot(time_data_u, u_sim_all[:self.data_idx - 1, 7] * 180 / np.pi, label="$\\alpha_{c4}$",
                        linestyle="--")
            ax2_right.set_ylabel("Servo Angle Cmd. ($^\\circ$)", fontsize=label_size)
            plt.legend(framealpha=legend_alpha, ncol=4, bbox_to_anchor=(0.1, 0.00), loc="lower left")
            plt.xlim([-0.1, t_total_sim])
            # plt.ylim([-1.0, 1.0])  # -0.8, 0.8; -1.6, 1.6

        # plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        plt.tight_layout()
        fig.subplots_adjust(hspace=0.15)

        plt.show()

    def visualize_rpy(self, 
                      ocp_model_name: str, 
                      ts_sim: float,
                      t_total_sim: float):

        plt.style.use(["science", "grid"])

        # Font size
        plt.rcParams.update({'font.size': 11})      # Default is 10
        label_size = 14

        x_sim_all = self.x_sim_all

        # Timeseries
        time_data_x = np.arange(self.data_idx) * ts_sim

        # Set up plot
        fig = plt.figure(figsize=(3.5, 2.0))
        title = str(f"{ocp_model_name}")
        title = title.replace("_", r"\_")
        # fig.title(title)

        # Convert Quaternions into Euler Angles using tf
        euler = np.zeros((x_sim_all.shape[0], 3))
        for i in range(x_sim_all.shape[0]):
            qwxyz = x_sim_all[i, 6:10]
            qxyzw = np.concatenate((qwxyz[1:], qwxyz[:1]))
            euler[i, :] = tf.euler_from_quaternion(qxyzw, axes="sxyz")

        # Plot Euler Angles
        plt.plot([0, t_total_sim], [0.5, 0.5], label="ref", linestyle="-.")     # Plot reference as constant 0.5
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


class SensorVisualizer:
    def __init__(self, N_sim):
        self.gyro_sim_all = np.ndarray((N_sim, 3))

        self.ang_acc_real_all = np.ndarray((N_sim, 3))
        self.ang_acc_est_all = np.ndarray((N_sim, 3))

        self.sf_sim_all = np.ndarray((N_sim, 3))
        self.lin_acc_est_all = np.ndarray((N_sim, 3))

        self.data_idx = 0

    def update_gyro(self, i, gyro, ang_acc_est, ang_acc_real):
        self.gyro_sim_all[i, :] = gyro
        self.ang_acc_est_all[i, :] = ang_acc_est
        self.ang_acc_real_all[i, :] = ang_acc_real

    def vis_gyro(self, ts_sim: float, t_total_sim: float):
        plt.style.use(["science", "grid"])

        # Font size
        plt.rcParams.update({'font.size': 11})

        fig = plt.figure(figsize=(7, 4.5))

        # Timeseries
        time_data_x = np.arange(self.data_idx) * ts_sim

        # Plot Angular Velocity as States
        ax = plt.subplot(211)
        plt.plot(time_data_x, self.gyro_sim_all[:self.data_idx, 0], label="gyro_x")
        plt.plot(time_data_x, self.gyro_sim_all[:self.data_idx, 1], label="gyro_y")
        plt.plot(time_data_x, self.gyro_sim_all[:self.data_idx, 2], label="gyro_z")
        plt.ylabel("Gyro Angular Vel. (rad/s)")
        plt.legend(framealpha=legend_alpha, ncol=3, bbox_to_anchor=(0.0, 0.7), loc="lower left")

        # Plot Angular Acceleration as States and Estimates
        ax2 = plt.subplot(212, sharex=ax)
        plt.plot(time_data_x, self.ang_acc_real_all[:self.data_idx, 0], label="ang_acc_real_x")
        plt.plot(time_data_x, self.ang_acc_real_all[:self.data_idx, 1], label="ang_acc_real_y")
        plt.plot(time_data_x, self.ang_acc_real_all[:self.data_idx, 2], label="ang_acc_real_z")
        plt.plot(time_data_x, self.ang_acc_est_all[:self.data_idx, 0], label="ang_acc_est_x")
        plt.plot(time_data_x, self.ang_acc_est_all[:self.data_idx, 1], label="ang_acc_est_y")
        plt.plot(time_data_x, self.ang_acc_est_all[:self.data_idx, 2], label="ang_acc_est_z")
        plt.ylabel("Angular Acc. (rad/s^2)")
        plt.xlabel("Time (s)")
        plt.legend(framealpha=legend_alpha, ncol=3, bbox_to_anchor=(0.0, 0.7), loc="lower left")

        plt.xlim([-0.1, t_total_sim])

        plt.tight_layout()
        fig.subplots_adjust(hspace=0.15)

        plt.show()