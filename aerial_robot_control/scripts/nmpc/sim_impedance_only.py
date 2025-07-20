import copy
import time
import numpy as np
import argparse

from nmpc_tilt_mt.utils.nmpc_viz import Visualizer

from nmpc_tilt_mt.utils.fir_differentiator import FIRDifferentiator

from nmpc_tilt_mt.tilt_qd.tilt_qd_servo_thrust_dist import NMPCTiltQdServoThrustDist
from nmpc_tilt_mt.tilt_qd.tilt_qd_servo_dist_imp import NMPCTiltQdServoImpedance
from nmpc_tilt_mt.misc.nominal_impedance import NominalImpedance

np.random.seed(42)


def main(args):
    # ========== Init ==========
    # ---------- Simulator ----------
    if args.sim_model == 0:
        sim_nmpc = NMPCTiltQdServoThrustDist()
    elif args.sim_model == 1:
        sim_nmpc = NominalImpedance()

    # Get time constants
    if sim_nmpc.include_servo_model:
        t_servo_sim = sim_nmpc.phys.t_servo
    else:
        t_servo_sim = 0.0
    if sim_nmpc.include_thrust_model:
        t_rotor_sim = sim_nmpc.phys.t_rotor
    else:
        t_rotor_sim = 0.0

    ts_sim = 0.005  # or 0.001

    t_total_sim = 40.0
    if args.plot_type == 1:
        t_total_sim = 4.0
    if args.plot_type == 2:
        t_total_sim = 3.0

    N_sim = int(t_total_sim / ts_sim)

    # Sim solver
    sim_solver = sim_nmpc.create_acados_sim_solver(ts_sim, is_build=True)
    nx_sim = sim_solver.acados_sim.dims.nx

    # Disturbance Initialization
    disturb_init = np.zeros(6)

    # State Initialization
    x_init_sim = np.zeros(nx_sim)
    x_init_sim[6] = 1.0  # qw

    # ---------- Visualization ----------
    viz = Visualizer(
        "",
        N_sim,
        nx_sim,
        0,
        x_init_sim,
        tilt=False,
        include_servo_model=sim_nmpc.include_servo_model,
        include_thrust_model=sim_nmpc.include_thrust_model,
        include_cog_dist_model=sim_nmpc.include_cog_dist_model,
        include_cog_dist_est=True,
    )

    # ========== Run simulation ==========
    t_ctl = 0.0
    t_sensor = 0.0
    x_now_sim = x_init_sim
    for i in range(N_sim):
        # --------- Update time ---------
        t_now = i * ts_sim
        t_ctl += ts_sim
        t_sensor += ts_sim

        # --------- Update disturbance ---------
        disturb = copy.deepcopy(disturb_init)
        # Simulate random disturbance
        # disturb[2] = np.random.normal(1.0, 3.0)  # fz in N

        # Simulate fixed disturbance at singular points
        if 2.0 <= t_now < 7.0:
            disturb[0] = 5.0

        if 7.0 <= t_now < 12.0:
            disturb[0] = 5.0
            disturb[1] = -5.0

        if 12.0 <= t_now < 17.0:
            disturb[0] = 5.0
            disturb[1] = -5.0
            disturb[2] = -5.0

        if 20.0 <= t_now < 25.0:
            disturb[3] = 5.0

        if 25.0 <= t_now < 30.0:
            disturb[3] = 5.0
            disturb[4] = -5.0

        if 30.0 <= t_now < 35.0:
            disturb[3] = 5.0
            disturb[4] = -5.0
            disturb[5] = 5.0

        # --------- Update simulation ----------
        sim_solver.set("x", x_now_sim)
        sim_solver.set("u", disturb)

        status = sim_solver.solve()
        if status != 0:
            raise Exception(f"acados integrator returned status {status} in closed loop instance {i}")

        x_now_sim = sim_solver.get("x")

        # --------- Update visualizer ----------
        viz.update(i, x_now_sim, 0)  # Note: The recording frequency of u_cmd is the same as ts_sim
        viz.update_est_disturb(i, disturb[0:3], disturb[3:6])

    # ========== Visualize ==========
    if args.plot_type == 0:
        viz.visualize(
            "no_mdl", sim_solver.model_name, 0.0, ts_sim, t_total_sim, t_servo_ctrl=0.0, t_servo_sim=t_servo_sim
        )
    elif args.plot_type == 1:
        viz.visualize_less(ts_sim, t_total_sim)
    elif args.plot_type == 2:
        viz.visualize_rpy("no_mdl", ts_sim, t_total_sim)
    elif args.plot_type == 3:
        viz.visualize_disturb(ts_sim, t_total_sim)
    elif args.plot_type == 4:
        viz.visualize_nothing_but_save("no_mdl", sim_solver.model_name)


if __name__ == "__main__":
    # Read command line arguments
    parser = argparse.ArgumentParser(description="Run the simulation of different NMPC models with impedance control.")

    parser.add_argument(
        "-sim",
        "--sim_model",
        type=int,
        default=0,
        help="The simulation model. " "Options: 0 (default: servo+thrust+dist), 1 (pure impedance).",
    )

    parser.add_argument(
        "-p",
        "--plot_type",
        type=int,
        default=0,
        help="The type of plot. " "Options: 0 (default: full), 1 (less), 2 (only rpy).",
    )

    args = parser.parse_args()
    main(args)
