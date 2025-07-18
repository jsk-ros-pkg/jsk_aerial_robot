import numpy as np

def apply_cog_disturbance(rtnmpc, state):
    """
    Function to generate a random disturbance force and torque on the center of gravity (CoG).
    This is a placeholder function that can be expanded with specific disturbance parameters.
    """
    # Force values behave in [-thrust_max, thrust_max]
    z = state[2]
    force_mu = max(1 / (z**2 + 1e-8), 0.9 * rtnmpc.nmpc.params["thrust_max"])
    force_std = 0.05 * rtnmpc.nmpc.params["thrust_max"]
    
    # Torque values behave in [-2, 2] with thrust_max = 30
    torque_mu = 0.5
    torque_std = 0.05 * 1/15 * rtnmpc.nmpc.params["thrust_max"]

    mu = np.array([force_mu, force_mu, force_mu,
                   torque_mu, torque_mu, torque_mu])
    std = np.array([force_std, force_std, force_std,
                    torque_std, torque_std, torque_std])
    cog_dist = np.random.normal(loc=mu, scale=std)
    rtnmpc.acados_parameters[28:34] = cog_dist


def apply_motor_noise(rtnmpc, u_cmd):
    if rtnmpc.nmpc.include_cog_dist_parameter:
        idx = 34
    else:
        idx = 28
    if rtnmpc.nmpc.include_impedance:
        raise NotImplementedError("Adjust indices for parameters.")

    # Thrust noise
    if u_cmd is None:
        thrust_factor = np.ones((4,))
    else:
        # Use last thrust command for normalization
        thrust_factor = u_cmd[:4] / rtnmpc.nmpc.params["thrust_max"]
    amplitude_mu = 0.06 * thrust_factor ** 2
    amplitude_std = 0.08
    mu = np.random.uniform(-amplitude_mu, amplitude_mu)
    std = amplitude_std * np.abs(thrust_factor) ** (1/4)
    rotor_noise = np.random.normal(loc=mu, scale=std)

    rtnmpc.acados_parameters[idx:idx+4] = rotor_noise

    # Servo angle noise
    if rtnmpc.nmpc.tilt:
        mu = np.zeros((4,))
        std = 0.04  # Assume constant inaccuracy for servo angles since they have low frequency
        servo_noise = np.random.normal(loc=mu, scale=std)

        rtnmpc.acados_parameters[idx+4:idx+8] = servo_noise