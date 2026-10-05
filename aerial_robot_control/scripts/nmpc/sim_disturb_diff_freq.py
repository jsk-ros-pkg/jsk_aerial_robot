import copy
import time
import numpy as np
import argparse

from nmpc_tilt_mt.utils.nmpc_viz import Visualizer
from nmpc_tilt_mt.utils.fir_differentiator import FIRDifferentiator
from nmpc_tilt_mt.tilt_qd.tilt_qd_servo_thrust_dist import NMPCTiltQdServoThrustDist  # Simulation
from nmpc_tilt_mt.tilt_qd.tilt_qd_servo_dist import NMPCTiltQdServoDist  # Control

np.random.seed(42)


def main(args):
    # ========== Init ==========
    # ---------- Controller ----------
    nmpc = NMPCTiltQdServoDist()

    t_servo_ctrl = nmpc.phys.t_servo
    ts_ctrl = nmpc.params["T_samp"]

    # OCP solver
    ocp_solver = nmpc.get_ocp_solver()
    nx = ocp_solver.acados_ocp.dims.nx
    nu = ocp_solver.acados_ocp.dims.nu
    n_param = ocp_solver.acados_ocp.dims.np

    x_init = np.zeros(nx)
    x_init[6] = 1.0  # qw
    u_init = np.zeros(nu)

    for stage in range(ocp_solver.N + 1):
        ocp_solver.set(stage, "x", x_init)
    for stage in range(ocp_solver.N):
        ocp_solver.set(stage, "u", u_init)

    # --------- Disturbance Rejection ---------
    ts_sensor = 0.0025
    disturb_estimated = np.zeros(6)  # fds_w, tau_ds_b. Note that, they are in different frames.
    disturb_nmpc_compd = np.zeros(6)  # fds_w, tau_ds_b. The disturbance that has been compensated by the nmpc.

    # ---------- Simulator ----------
    sim_nmpc = NMPCTiltQdServoThrustDist()

    # Get time constants
    t_servo_sim = sim_nmpc.phys.t_servo
    t_rotor_sim = sim_nmpc.phys.t_rotor

    ts_sim = 0.005  # or 0.001

    t_total_sim = 15.0
    if args.plot_type == 1:
        t_total_sim = 4.0
    if args.plot_type == 2:
        t_total_sim = 3.0

    N_sim = int(t_total_sim / ts_sim)

    # Sim solver
    sim_solver = sim_nmpc.create_acados_sim_solver(ts_sim, build=True)
    nx_sim = sim_solver.acados_sim.dims.nx

    # Disturbance Initialization
    disturb_init = np.zeros(6)
    # disturb_init[0] = 1.0
    # disturb_init[1] = -1.0
    # disturb_init[2] = 1.0  # N, fz
    # disturb_init[3] = 0.1
    # disturb_init[4] = -0.1
    # disturb_init[5] = 0.1

    # State Initialization
    x_init_sim = np.zeros(nx_sim)
    x_init_sim[6] = 1.0  # qw
    x_init_sim[-6:] = disturb_init

    # ---------- Reference ----------
    reference_generator = nmpc.get_reference_generator()

    # ---------- Visualization ----------
    viz = Visualizer(
        "qd",
        N_sim,
        nx_sim,
        nu,
        x_init_sim,
        tilt=nmpc.tilt,
        include_servo_model=sim_nmpc.include_servo_model,
        include_thrust_model=sim_nmpc.include_thrust_model,
        include_cog_dist_model=sim_nmpc.include_cog_dist_model,
        is_record_diff_u=True,
    )

    # ---------- Sensors ----------
    fir_param = np.array([-1, 8, 0, -8, 1]) / 12  # 5-point FIR differentiator
    gyro_differentiator = [
        FIRDifferentiator(fir_param, ts_sensor),
        FIRDifferentiator(fir_param, ts_sensor),
        FIRDifferentiator(fir_param, ts_sensor),
    ]  # for gyro differentiation

    # ========== Run simulation ==========
    u_cmd = u_init
    u_mpc = u_init
    t_ctl = 0.0
    t_sensor = 0.0
    x_now_sim = x_init_sim
    for i in range(N_sim):
        # --------- Update time ---------
        t_now = i * ts_sim
        t_ctl += ts_sim
        t_sensor += ts_sim

        # --------- Update state estimation ---------
        # Assemble state from simulation and disturbance estimation
        if nmpc.include_cog_dist_model:
            x_now = np.zeros(nx)
            x_now[: nx - 6] = x_now_sim[: nx - 6]
            x_now[-6:] = disturb_estimated
        else:
            x_now = x_now_sim[:nx]  # The dimension of x_now may be smaller than x_now_sim

        # -------- Update control target --------
        target_xyz = np.array([[0.3, 0.6, 1.0]]).T
        target_rpy = np.array([[0.0, 0.0, 0.0]]).T

        if args.plot_type == 2:
            target_xyz = np.array([[0.0, 0.0, 0.0]]).T
            target_rpy = np.array([[0.5, 0.5, 0.5]]).T

        # if t_total_sim > 2.0:
        #     if 2.0 <= t_now < 6:
        #         target_xyz = np.array([[0.3, 0.6, 1.0]]).T
        #
        #         roll = 30.0 / 180.0 * np.pi
        #         pitch = 60.0 / 180.0 * np.pi
        #         yaw = 90.0 / 180.0 * np.pi
        #         target_rpy = np.array([[roll, pitch, yaw]]).T
        #
        #     if t_now >= 6:
        #         assert t_sqp_end <= 3.0
        #         target_xyz = np.array([[1.0, 1.0, 1.0]]).T
        #         target_rpy = np.array([[0.0, 0.0, 0.0]]).T

        # Compute reference trajectory from target pose
        xr, ur = reference_generator.compute_trajectory(target_xyz, target_rpy)

        # -------- Update solver --------
        comp_time_start = time.time()

        if t_ctl >= ts_ctrl:
            t_ctl = 0.0

            # 0 ~ N-1
            for j in range(ocp_solver.N):
                yr = np.concatenate((xr[j, :], ur[j, :]))
                ocp_solver.set(j, "yref", yr)
                quaternion_r = xr[j, 6:10]
                nmpc.acados_init_p[0:4] = quaternion_r
                ocp_solver.set(j, "p", nmpc.acados_init_p)  # For nonlinear quaternion error

            # N
            yr = xr[ocp_solver.N, :]
            ocp_solver.set(ocp_solver.N, "yref", yr)  # Final state of x, no u
            quaternion_r = xr[ocp_solver.N, 6:10]
            nmpc.acados_init_p[0:4] = quaternion_r
            ocp_solver.set(ocp_solver.N, "p", nmpc.acados_init_p)  # For nonlinear quaternion error

            # Compute control feedback and take the first action
            try:
                u_mpc = ocp_solver.solve_for_x0(x_now)
                disturb_nmpc_compd = copy.deepcopy(disturb_estimated)
            except Exception as e:
                print(f"Round {i}: acados ocp_solver returned status {ocp_solver.status}. Exiting.")
                break

        comp_time_end = time.time()
        viz.comp_time[i] = comp_time_end - comp_time_start

        # -------- Sensor-based disturbance rejection --------
        if hasattr(viz, "u_sim_mpc_all"):
            viz.update_u_mpc(i, u_mpc)

        # By default, the u_cmd is the mpc command
        u_cmd = copy.deepcopy(u_mpc)

        # Disturbance estimation is related to the sensor update frequency
        if t_sensor >= ts_sensor:
            t_sensor = 0.0

            # Calculate the internal wrench from IMU measurements in Body frame
            sf_b, ang_acc_b, rot_wb = sim_nmpc.fake_sensor.update_acc(x_now_sim)

            w = x_now_sim[10:13]  # Angular velocity
            mass = sim_nmpc.fake_sensor.mass
            gravity = sim_nmpc.fake_sensor.gravity
            I = sim_nmpc.fake_sensor.I

            sf_b_imu = sf_b + np.random.normal(0.0, 0.1, 3)  # add noise. real: scale = 0.00727 * gravity
            w_imu = w + np.random.normal(0.0, 0.001, 3)  # add noise. real: scale = 0.0008 rad/s

            ang_acc_b_imu = np.zeros(3)
            if args.if_use_ang_acc == 0:
                ang_acc_b_imu[0] = gyro_differentiator[0].apply_single(w_imu[0])
                ang_acc_b_imu[1] = gyro_differentiator[1].apply_single(w_imu[1])
                ang_acc_b_imu[2] = gyro_differentiator[2].apply_single(w_imu[2])
            else:
                ang_acc_b_imu = ang_acc_b

            wrench_u_imu_b = np.zeros(6)
            wrench_u_imu_b[0:3] = mass * sf_b_imu
            wrench_u_imu_b[3:6] = np.dot(I, ang_acc_b_imu) + np.cross(w, np.dot(I, w))

            # Calculate the internal wrench from actuator sensor measurements in Body frame
            ft_sensor = x_now_sim[17:21]
            a_sensor = x_now_sim[13:17]

            z_sensor = np.zeros(8)
            z_sensor[0] = ft_sensor[0] * np.sin(a_sensor[0])
            z_sensor[1] = ft_sensor[0] * np.cos(a_sensor[0])
            z_sensor[2] = ft_sensor[1] * np.sin(a_sensor[1])
            z_sensor[3] = ft_sensor[1] * np.cos(a_sensor[1])
            z_sensor[4] = ft_sensor[2] * np.sin(a_sensor[2])
            z_sensor[5] = ft_sensor[2] * np.cos(a_sensor[2])
            z_sensor[6] = ft_sensor[3] * np.sin(a_sensor[3])
            z_sensor[7] = ft_sensor[3] * np.cos(a_sensor[3])

            wrench_u_sensor_b = np.dot(reference_generator.get_alloc_mat(), z_sensor)

            u_meas = np.zeros(8)
            u_meas[0:4] = ft_sensor
            u_meas[4:] = a_sensor

            # Update disturbance estimation
            if args.if_est_dist:
                # Only use the wrench difference between the imu and the actuator sensor, no u_mpc
                lpf_freq = 2.0  # Hz
                tau = 1 / (2 * np.pi * lpf_freq)
                alpha = ts_sensor / (
                    tau + ts_sensor
                )  # Low-pass filter coefficient. Note that, it is related to the sensor update frequency.

                disturb_estimated[0:3] = (1 - alpha) * disturb_estimated[0:3] + alpha * np.dot(
                    rot_wb, (wrench_u_imu_b[0:3] - wrench_u_sensor_b[0:3])
                )  # World frame
                disturb_estimated[3:6] = (1 - alpha) * disturb_estimated[3:6] + alpha * (
                    wrench_u_imu_b[3:6] - wrench_u_sensor_b[3:6]
                )  # Body frame

        # --------- Update simulation ----------
        disturb = copy.deepcopy(disturb_init)

        # sweep frequency of disturbance
        dist_freq = 5  # Hz
        disturb[0] = 5.0 * np.sin(2 * np.pi * dist_freq * t_now)  # fx in N
        # Simulate random disturbance
        # disturb[2] = np.random.normal(1.0, 3.0)  # fz in N

        # Simulate fixed disturbance at singular points
        # if 2.0 <= t_now < 3.0:
        #     disturb[0] = 5.0
        #     disturb[1] = -5.0
        #     disturb[2] = -5.0
        #
        # if 5.0 <= t_now < 6.0:
        #     disturb[3] = 0.5
        #     disturb[4] = -0.5
        #     disturb[5] = 0.5

        x_now_sim[-6:] = disturb

        sim_solver.set("x", x_now_sim)
        sim_solver.set("u", u_cmd)

        status = sim_solver.solve()
        if status != 0:
            raise Exception(f"acados integrator returned status {status} in closed loop instance {i}")

        x_now_sim = sim_solver.get("x")

        # --------- Update visualizer ----------
        viz.update(i, x_now_sim, u_cmd)  # Note: The recording frequency of u_cmd is the same as ts_sim

    # ========== Visualize ==========
    if args.plot_type == 0:
        viz.visualize(
            ocp_solver.acados_ocp.model.name,
            sim_solver.model_name,
            ts_ctrl,
            ts_sim,
            t_total_sim,
            t_servo_ctrl=t_servo_ctrl,
            t_servo_sim=t_servo_sim,
        )
    elif args.plot_type == 1:
        viz.visualize_less(ts_sim, t_total_sim)
    elif args.plot_type == 2:
        viz.visualize_rpy(ocp_solver.acados_ocp.model.name, ts_sim, t_total_sim)


if __name__ == "__main__":
    # Read command line arguments
    parser = argparse.ArgumentParser(description="Run the simulation of different disturbance rejection methods.")
    parser.add_argument(
        "--if_est_dist", type=int, default=1, help="Whether to estimate the disturbance. Options: 0 (no), 1 (yes)."
    )

    parser.add_argument(
        "-p",
        "--plot_type",
        type=int,
        default=0,
        help="The type of plot. " "Options: 0 (default: full), 1 (less), 2 (only rpy).",
    )

    parser.add_argument(
        "-b",
        "--if_use_ang_acc",
        type=int,
        default=1,
        help="Flag to use ground truth angular acceleration. Default: 1 (True)",
    )

    args = parser.parse_args()

    main(args)
