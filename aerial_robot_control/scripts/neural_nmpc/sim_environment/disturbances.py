import numpy as np
from acados_template import AcadosSimSolver
from neural_controller import NeuralNMPC


def apply_cog_disturbance(sim_solver: AcadosSimSolver, rtnmpc: NeuralNMPC, cog_dist_factor, u_cmd, state):
    """
    Function to generate a random disturbance force and torque on the center of gravity (CoG).
    This is a placeholder function that can be expanded with specific disturbance parameters.
    """
    # Get average of thrust command for smoother disturbance
    # u_sequence = np.array([rtnmpc.ocp_solver.get(i, "u") for i in range(int(rtnmpc.N/5))])
    # max_thrust = np.average(u_sequence[:,:4])
    max_thrust = np.average(u_cmd[:4])
    # Ground effect increases lift the closer drone is to the ground
    # Force values behave in [-thrust_max, thrust_max]
    z = state[2]
    # force_mu_z = min(1 / (z+1)**2, 1) * 0.3 * max_thrust * 4
    force_mu_z = 1 / (z + 1) ** 2 * cog_dist_factor * max_thrust * 4
    force_std_z = 0  # 0.01 * max_thrust

    force_mu_x = 0.0
    force_mu_y = 0.0
    force_std_x = 0  # 0.05 * max_thrust
    force_std_y = 0  # 0.05 * max_thrust

    # Torque values behave in [-2, 2], with thrust_max = 30
    torque_mu = 0  # 0.5
    torque_std = 0  # 0.05 * 1/15 * max_thrust

    mu = np.array([force_mu_x, force_mu_y, force_mu_z, torque_mu, torque_mu, torque_mu])
    std = np.array([force_std_x, force_std_y, force_std_z, torque_std, torque_std, torque_std])
    cog_dist = mu  # np.random.normal(loc=mu, scale=std)
    start_idx = rtnmpc.cog_dist_start_idx
    end_idx = rtnmpc.cog_dist_end_idx
    sim_solver.acados_sim.parameter_values[start_idx:end_idx] = cog_dist


def apply_motor_noise(sim_solver: AcadosSimSolver, rtnmpc: NeuralNMPC, u_cmd):
    # Thrust noise
    if u_cmd is None:
        thrust_factor = np.ones((4,))
    else:
        # Use last thrust command for normalization
        thrust_factor = u_cmd[:4] / rtnmpc.params["thrust_max"]
    amplitude_mu = 0.06 * thrust_factor**2
    amplitude_std = 0.08
    mu = np.random.uniform(-amplitude_mu, amplitude_mu)
    std = np.abs(thrust_factor) * amplitude_std ** (1 / 4)
    rotor_noise = np.random.normal(loc=mu, scale=std)

    start_idx = rtnmpc.motor_noise_start_idx
    end_idx = rtnmpc.motor_noise_end_idx
    sim_solver.acados_sim.parameter_values[start_idx : start_idx + 4] = rotor_noise

    # Servo angle noise
    if rtnmpc.nmpc.tilt:
        mu = np.zeros((4,))
        std = 0.04  # Assume constant inaccuracy for servo angles since they have low frequency
        servo_noise = np.random.normal(loc=mu, scale=std)

        sim_solver.acados_sim.parameter_values[start_idx + 4 : end_idx] = servo_noise
