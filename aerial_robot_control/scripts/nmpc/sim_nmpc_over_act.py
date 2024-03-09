'''
 Created by li-jinjie on 24-3-9.
'''
import numpy as np
import matplotlib.pyplot as plt
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver, AcadosSim, AcadosSimSolver
import casadi as ca
from tf_conversions import transformations as tf

from nmpc_over_act_full import create_acados_ocp_solver, XrUrConverter


def create_acados_sim_solver(ocp_model: AcadosModel, ts_sim: float) -> AcadosSimSolver:
    acados_sim = AcadosSim()
    acados_sim.model = ocp_model
    n_params = ocp_model.p.size()[0]
    acados_sim.dims.np = n_params  # TODO: seems that the np needn't to be set manually in the latest version of acados
    acados_sim.parameter_values = np.zeros(n_params)
    acados_sim.solver_options.T = ts_sim
    acados_sim_solver = AcadosSimSolver(acados_sim, json_file="acados_ocp_" + ocp_model.name + ".json")
    return acados_sim_solver


class Visualizer:
    def __init__(self, N_sim, nx, nu, x0):
        self.x_sim_all = np.ndarray((N_sim + 1, nx))
        self.u_sim_all = np.ndarray((N_sim, nu))
        self.x_sim_all[0, :] = x0

    def update(self, i, x, u):
        self.x_sim_all[i + 1, :] = x
        self.u_sim_all[i, :] = u

    def visualize(self, ts_ctrl: float, t_servo: float = 0.0, t_sqp_start: float = 0, t_sqp_end: float = 0):
        x_sim_all = self.x_sim_all
        u_sim_all = self.u_sim_all

        fig = plt.figure(figsize=(20, 10))
        fig.suptitle(
            f"New u cost NMPC closed-loop sim with ts_sim = {ts_sim} s and ts_ctrl = {ts_ctrl} s\n"
            f"servo delay {t_servo} s"
        )

        plt.subplot(4, 2, 1)
        plt.plot(np.arange(x_sim_all.shape[0]) * ts_sim, x_sim_all[:, 0], label="x")
        plt.plot(np.arange(x_sim_all.shape[0]) * ts_sim, x_sim_all[:, 1], label="y")
        plt.plot(np.arange(x_sim_all.shape[0]) * ts_sim, x_sim_all[:, 2], label="z")
        plt.legend()
        plt.xlabel("time (s)")
        plt.ylabel("position (m)")
        plt.axvspan(t_sqp_start, t_sqp_end, facecolor="orange", alpha=0.2)
        plt.text(1.5, 0.5, "SQP_RTI", horizontalalignment="center", verticalalignment="center")
        plt.text((t_sqp_start + t_sqp_end) / 2, 0.5, "SQP", horizontalalignment="center", verticalalignment="center")
        plt.text(4.0, 0.5, "SQP_RTI", horizontalalignment="center", verticalalignment="center")
        plt.grid(True)

        plt.subplot(4, 2, 3)
        plt.plot(np.arange(x_sim_all.shape[0]) * ts_sim, x_sim_all[:, 3], label="vx")
        plt.plot(np.arange(x_sim_all.shape[0]) * ts_sim, x_sim_all[:, 4], label="vy")
        plt.plot(np.arange(x_sim_all.shape[0]) * ts_sim, x_sim_all[:, 5], label="vz")
        plt.legend()
        plt.xlabel("time (s)")
        plt.ylabel("velocity (m/s)")
        plt.grid(True)
        plt.axvspan(t_sqp_start, t_sqp_end, facecolor="orange", alpha=0.2)

        plt.subplot(4, 2, 5)
        plt.plot(np.arange(x_sim_all.shape[0]) * ts_sim, x_sim_all[:, 6], label="qw")
        plt.plot(np.arange(x_sim_all.shape[0]) * ts_sim, x_sim_all[:, 7], label="qx")
        plt.plot(np.arange(x_sim_all.shape[0]) * ts_sim, x_sim_all[:, 8], label="qy")
        plt.plot(np.arange(x_sim_all.shape[0]) * ts_sim, x_sim_all[:, 9], label="qz")
        plt.legend()
        plt.xlabel("time (s)")
        plt.ylabel("quaternion")
        plt.grid(True)
        plt.axvspan(t_sqp_start, t_sqp_end, facecolor="orange", alpha=0.2)

        plt.subplot(4, 2, 7)
        # use tf2 to convert x_sim_all[:, 6:10] to euler angle
        euler = np.zeros((x_sim_all.shape[0], 3))
        for i in range(x_sim_all.shape[0]):
            qwxyz = x_sim_all[i, 6:10]
            qxyzw = np.concatenate((qwxyz[1:], qwxyz[:1]))
            euler[i, :] = tf.euler_from_quaternion(qxyzw, axes="sxyz")

        plt.plot(np.arange(x_sim_all.shape[0]) * ts_sim, euler[:, 0], label="roll")
        plt.plot(np.arange(x_sim_all.shape[0]) * ts_sim, euler[:, 1], label="pitch")
        plt.plot(np.arange(x_sim_all.shape[0]) * ts_sim, euler[:, 2], label="yaw")
        plt.legend()
        plt.xlabel("time (s)")
        plt.ylabel("euler angle (rad)")
        plt.grid(True)
        plt.axvspan(t_sqp_start, t_sqp_end, facecolor="orange", alpha=0.2)

        plt.subplot(4, 2, 2)
        plt.plot(np.arange(x_sim_all.shape[0]) * ts_sim, x_sim_all[:, 10], label="wx")
        plt.plot(np.arange(x_sim_all.shape[0]) * ts_sim, x_sim_all[:, 11], label="wy")
        plt.plot(np.arange(x_sim_all.shape[0]) * ts_sim, x_sim_all[:, 12], label="wz")
        plt.legend()
        plt.xlabel("time (s)")
        plt.ylabel("body rate (rad/s)")
        plt.grid(True)
        plt.axvspan(t_sqp_start, t_sqp_end, facecolor="orange", alpha=0.2)

        plt.subplot(4, 2, 4)
        plt.plot(np.arange(x_sim_all.shape[0]) * ts_sim, x_sim_all[:, 13], label="a1")
        plt.plot(np.arange(x_sim_all.shape[0]) * ts_sim, x_sim_all[:, 14], label="a2")
        plt.plot(np.arange(x_sim_all.shape[0]) * ts_sim, x_sim_all[:, 15], label="a3")
        plt.plot(np.arange(x_sim_all.shape[0]) * ts_sim, x_sim_all[:, 16], label="a4")
        plt.legend()
        plt.xlabel("time (s)")
        plt.ylabel("servo angle (rad)")
        plt.grid(True)
        plt.axvspan(t_sqp_start, t_sqp_end, facecolor="orange", alpha=0.2)

        plt.subplot(4, 2, 6)
        plt.plot(np.arange(u_sim_all.shape[0]) * ts_sim, u_sim_all[:, 0], label="ft1")
        plt.plot(np.arange(u_sim_all.shape[0]) * ts_sim, u_sim_all[:, 1], label="ft2")
        plt.plot(np.arange(u_sim_all.shape[0]) * ts_sim, u_sim_all[:, 2], label="ft3")
        plt.plot(np.arange(u_sim_all.shape[0]) * ts_sim, u_sim_all[:, 3], label="ft4")
        plt.legend()
        plt.xlabel("time (s)")
        plt.ylabel("thrust (N)")
        plt.grid(True)
        plt.axvspan(t_sqp_start, t_sqp_end, facecolor="orange", alpha=0.2)

        plt.subplot(4, 2, 8)
        plt.plot(np.arange(u_sim_all.shape[0]) * ts_sim, u_sim_all[:, 4], label="a1c")
        plt.plot(np.arange(u_sim_all.shape[0]) * ts_sim, u_sim_all[:, 5], label="a2c")
        plt.plot(np.arange(u_sim_all.shape[0]) * ts_sim, u_sim_all[:, 6], label="a3c")
        plt.plot(np.arange(u_sim_all.shape[0]) * ts_sim, u_sim_all[:, 7], label="a4c")
        plt.legend()
        plt.xlabel("time (s)")
        plt.ylabel("servo angle cmd (rad)")
        plt.grid(True)
        plt.axvspan(t_sqp_start, t_sqp_end, facecolor="orange", alpha=0.2)

        plt.tight_layout(rect=[0, 0.03, 1, 0.95])

        plt.show()


if __name__ == "__main__":
    # simulation parameters
    ts_sim = 0.005
    t_total_sim = 15.0
    N_sim = int(t_total_sim / ts_sim)

    # acados ocp solver
    ocp_solver = create_acados_ocp_solver()
    t_servo = 0.085883  # TODO: wrap these parameters into a class
    ts_ctrl = 0.01

    nx = ocp_solver.acados_ocp.dims.nx
    nu = ocp_solver.acados_ocp.dims.nu
    sim_solver = create_acados_sim_solver(ocp_solver.acados_ocp.model, ts_sim)

    # initial states and control inputs
    x_init = np.zeros(nx)
    x_init[6] = 1.0  # qw
    u_init = np.zeros(nu)

    x_current = x_init
    u0 = u_init

    # - viz
    viz = Visualizer(N_sim, nx, nu, x_init)

    # - ocp solver
    for stage in range(ocp_solver.N + 1):
        ocp_solver.set(stage, "x", x_init)
    for stage in range(ocp_solver.N):
        ocp_solver.set(stage, "u", u_init)

    # - planner
    xr_ur_converter = XrUrConverter()

    # ========== update ==========
    t_sqp_start = 2.5
    t_sqp_end = 3.0

    t_ctl = 0.0
    for i in range(N_sim):
        t_now = i * ts_sim
        t_ctl += ts_sim

        # -------- sqp mode --------
        if t_now >= t_sqp_start:
            ocp_solver.solver_options["nlp_solver_type"] = "SQP"

        if t_now >= t_sqp_end:
            ocp_solver.solver_options["nlp_solver_type"] = "SQP_RTI"

        # -------- update target --------
        target_xyz = np.zeros((3, 1))
        target_rpy = np.zeros((3, 1))

        if 0.0 <= t_now < 3.0:
            target_xyz = np.array([[0.0, 0.0, 1.0]]).T
            target_rpy = np.array([[0.0, 0.0, 0.0]]).T

        if 3.0 <= t_now < 5.5:
            assert t_sqp_end <= 3.0
            target_xyz = np.array([[1.0, 1.0, 1.0]]).T
            target_rpy = np.array([[0.0, 0.0, 0.0]]).T

        if t_now >= 5.5:
            target_xyz = np.array([[1.0, 1.0, 1.0]]).T

            roll = 30.0 / 180.0 * np.pi
            pitch = 0.0 / 180.0 * np.pi
            yaw = 0.0 / 180.0 * np.pi
            target_rpy = np.array([[roll, pitch, yaw]]).T

        xr, ur = xr_ur_converter.update(target_xyz, target_rpy)

        # -------- update solver --------
        if t_ctl >= ts_ctrl:
            t_ctl = 0.0

            for j in range(ocp_solver.N):
                yr = np.concatenate((xr[j, :], ur[j, :]))
                ocp_solver.set(j, "yref", yr)

                quaternion_r = xr[j, 6:10]
                ocp_solver.set(j, "p", quaternion_r)  # for nonlinear quaternion error
            ocp_solver.set(ocp_solver.N, "yref", xr[ocp_solver.N, :])  # final state of x, no u

            quaternion_r = xr[ocp_solver.N, 6:10]
            ocp_solver.set(ocp_solver.N, "p", quaternion_r)  # for nonlinear quaternion error

            # feedback, take the first action
            u0 = ocp_solver.solve_for_x0(x_current)
            if ocp_solver.status != 0:
                raise Exception(
                    "acados ocp_solver returned status {}. Exiting.".format(ocp_solver.status)
                )

        # --------- update simulation ----------
        sim_solver.set("x", x_current)
        sim_solver.set("u", u0)
        status = sim_solver.solve()
        if status != 0:
            raise Exception(f"acados integrator returned status {status} in closed loop instance {i}")

        # update state
        x_current = sim_solver.get("x")

        viz.update(i, x_current, u0)

    # visualize
    viz.visualize(ts_ctrl, t_servo, t_sqp_start, t_sqp_end)
