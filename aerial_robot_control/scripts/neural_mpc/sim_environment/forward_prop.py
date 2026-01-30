import os, sys
import numpy as np
import casadi as ca

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from utils.geometry_utils import skew_symmetric
from neural_controller import NeuralMPC


def init_forward_prop(mpc: NeuralMPC):
    p = ca.MX.sym("p", 3)  # Position
    v = ca.MX.sym("v", 3)  # Linear velocity in World frame (inertial reference for Newton's laws)
    q = ca.MX.sym("q", 4)  # Quaternion (representing orientation of the Body frame relative to the World frame)
    w = ca.MX.sym("w", 3)  # Angular velocity in Body frame (principal axes, diagonal inertia matrix)
    state = ca.vertcat(p, v, q, w)

    # - Extend state-space by dynamics of servo angles
    if mpc.tilt and mpc.include_servo_model:
        a_s = ca.MX.sym("a_s", 4)
        state = ca.vertcat(state, a_s)

    # - Extend state-space by dynamics of rotors
    if mpc.include_thrust_model:
        ft_s = ca.MX.sym("ft_s", 4)
        state = ca.vertcat(state, ft_s)

    ft_c = ca.MX.sym("ft_c", 4)
    a_c = ca.MX.sym("a_c", 4)
    u = ca.vertcat(ft_c, a_c)

    # Generate symbolic CasADi function
    dynamics = full_dynamics(mpc, state, u)
    return dynamics, state, u


def forward_prop(discretized_dynamics, state_0, u_cmd, T_horizon, T_step):
    """
    Propagates the state forward given sequence of inputs for the system u_sym. These inputs can either be numerical or symbolic.
    :param u_cmd: Optimized N x nu sequence of control inputs from MPC scheme
    :param T_horizon: Time span between first and last control input in u_cmd (default is the time horizon of the MPC)
    :param T_step: Integration time for in between every control input in u_cmd. The number of integration steps N is
    :return: An m x n array of propagated state estimates
    """
    if T_horizon % T_step != 0:
        # print(f"[Warning] In forward propagation, T_horizon {T_horizon} is not a multiple of T_step {T_step}.")
        # print(f"Matching T_step to be a multiple of T_horizon.")
        T_step = T_horizon / 2

    # Initialize parameters
    N = int(T_horizon / T_step)  # Number of integration steps
    if N != 2:
        print(f"[Warning] In forward propagation, the number of integration steps N={N} is not 2.")
        raise NotImplementedError(
            "Currently, only N=2 is expected. Please be only move on when you know what you are doing."
        )

    # Initialize sequence of propagated states
    state_k = state_0.copy()
    state_prop = state_k.copy()

    # Assume constant input over the propagation time horizon
    u_cmd = np.tile(u_cmd, (N, 1))  # Repeat the control input for each time step

    # Propagate forward
    for k in range(N):
        u_k = u_cmd[k, :]
        state_k = np.array(discretized_dynamics(state_k, u_k))
        state_prop = np.append(state_prop, state_k.T, axis=0)

    return state_prop


def discretize_dynamics(dynamics, x, u, T_step, num_stages):
    """
    Integrates the symbolic dynamics until the time horizon using a RK4 method.
    :param dynamics: CasADi function representing the continuous system dynamics
    :param x: CasADi symbolic variable representing the state
    :param u: CasADi symbolic variable representing the control inputs
    :param T_step: Single step size of forward integration in seconds
    :param num_stages: Number of stages in the Runge-Kutta integrator
    computed as T_horizon / T_step.
    :param num_stages: Number of stages in the Runge-Kutta integrator. Default is 1, i.e., the control inputs are
    applied once at the beginning of each integration step.
    :return: CasADi function that computes the next state x_k+1 based from previous state x_k and
    control input u_k over time T_step
    """
    x0 = x  # NOTE: SX/MX datatypes are immutable
    dt = T_step / num_stages

    # Fixed step Runge-Kutta 4 integrator
    for j in range(num_stages):
        m1 = dynamics(x=x, u=u)["x_dot"]
        m2 = dynamics(x=x + dt / 2 * m1, u=u)["x_dot"]
        m3 = dynamics(x=x + dt / 2 * m2, u=u)["x_dot"]
        m4 = dynamics(x=x + dt * m3, u=u)["x_dot"]
        x += dt / 6 * (m1 + 2 * m2 + 2 * m3 + m4)

    return ca.Function("F", [x0, u], [x], ["x0", "u"], ["xf"])


def linearized_dynamics(state, input, nx, nu, mass, I):
    raise NotImplementedError("linearized_dynamics is not implemented correctly and not used.")
    """
    Jacobian J matrix of the linearized dynamics specified in the function quad_dynamics. J[i, j] corresponds to
    the partial derivative of f_i(x) wrt x(j).

    :return: a CasADi symbolic function that calculates the nx x nx Jacobian matrix of the linearized simplified
    quadrotor dynamics
    """
    if isinstance(state, ca.MX):
        cs_type = ca.MX
    elif isinstance(state, ca.SX):
        cs_type = ca.SX

    # TODO state is the casadi symbolic state vector from the mpc object
    p = state[0:3]  # Position
    v = state[3:6]  # Linear velocity
    q = state[6:10]  # Quaternion
    w = state[10:13]  # Angular velocity

    u = input

    jac = cs_type(nx, nx)

    # Position derivatives
    jac[0:3, 3:6] = ca.diag(cs_type.ones(3))

    # Quaternion derivatives
    jac[6:10, 6:10] = skew_symmetric(w) / 2
    jac[6, 10:13] = 1 / 2 * ca.horzcat(-q[1], -q[2], -q[3])
    jac[7, 10:13] = 1 / 2 * ca.horzcat(q[0], -q[3], q[2])
    jac[8, 10:13] = 1 / 2 * ca.horzcat(q[3], q[0], -q[1])
    jac[9, 10:13] = 1 / 2 * ca.horzcat(-q[2], q[1], q[0])

    # Velocity derivatives
    a_u = (u[0] + u[1] + u[2] + u[3]) / mass
    jac[3, 6:10] = 2 * ca.horzcat(a_u * q[2], a_u * q[3], a_u * q[0], a_u * q[1])
    jac[4, 6:10] = 2 * ca.horzcat(-a_u * q[1], -a_u * q[0], a_u * q[3], a_u * q[2])
    jac[5, 6:10] = 2 * ca.horzcat(0, -2 * a_u * q[1], -2 * a_u * q[1], 0)

    # Rate derivatives
    jac[10, 10:13] = (I[1] - I[2]) / I[0] * ca.horzcat(0, w[2], w[1])
    jac[11, 10:13] = (I[2] - I[0]) / I[1] * ca.horzcat(w[2], 0, w[0])
    jac[12, 10:13] = (I[0] - I[1]) / I[2] * ca.horzcat(w[1], w[0], 0)

    return ca.Function("J", [state, u], [jac])


def full_dynamics(mpc: NeuralMPC, state, u):
    """
    Continuous-time dynamics of a omni-directional quadrotor. The model is based on the influence of the rotor
    thrusts at each arm on the total force and torque acting on the CoG.

    TODO INCLUDE :param rdrv_d: a 3x3 diagonal matrix containing the D matrix coefficients for a linear drag model as proposed
    by Faessler et al.

    :return: CasADi function that computes the analytical differential state dynamics of the quadrotor model
    Inputs: 'x' state of quadrotor (nx x 1) and 'u' control input (nu x 1)
    Output: Differential state vector 'x_dot' (nx x 1)
    """
    p = state[0:3]
    v = state[3:6]
    q = state[6:10]
    qw = state[6]
    qx = state[7]
    qy = state[8]
    qz = state[9]
    w = state[10:13]
    if mpc.tilt and mpc.include_servo_model:
        a_s = state[13:17]
    if mpc.include_thrust_model:
        ft_s = state[17:21]

    ft_c = u[:4]
    if mpc.tilt:
        a_c = u[4:8]

    # Physical parameters
    mass = mpc.phys.mass
    [Ixx, Iyy, Izz] = [mpc.phys.Ixx, mpc.phys.Iyy, mpc.phys.Izz]
    I = ca.diag(ca.vertcat(Ixx, Iyy, Izz))
    I_inv = ca.diag(ca.vertcat(1 / Ixx, 1 / Iyy, 1 / Izz))
    g_w = ca.vertcat(0, 0, -mpc.phys.gravity)

    t_rotor = mpc.phys.t_rotor  # Time constant of rotor dynamics
    t_servo = mpc.phys.t_servo  # Time constant of servo dynamics

    p_b = ca.horzcat(mpc.phys.p1_b, mpc.phys.p2_b, mpc.phys.p3_b, mpc.phys.p4_b).T

    dr = ca.vertcat(mpc.phys.dr1, mpc.phys.dr2, mpc.phys.dr3, mpc.phys.dr4)
    kq_d_kt = mpc.phys.kq_d_kt

    ee_pos = mpc.phys.ball_effector_p
    ee_q = mpc.phys.ball_effector_q

    # - Rotor to End-of-arm
    if mpc.tilt:
        if mpc.include_servo_model:
            a = a_s
        else:
            a = a_c
        cos_a = ca.cos(a)
        sin_a = ca.sin(a)
    else:
        cos_a = ca.MX.ones(4)
        sin_a = ca.MX.zeros(4)

    #     i < 3
    # u @Dschodscho

    # - End-of-arm to Body
    # Vectorized rotation matrices rot_be (3x3x4)
    # We'll compute the rotated forces later directly without storing the full rotation matrices
    norm_xy = ca.sqrt(p_b[:, 0] ** 2 + p_b[:, 1] ** 2)  # Avoid using norm_2()
    sin_theta = p_b[:, 1] / norm_xy
    cos_theta = p_b[:, 0] / norm_xy

    # - Body to World
    row_1 = ca.horzcat(1 - 2 * qy**2 - 2 * qz**2, 2 * qx * qy - 2 * qw * qz, 2 * qx * qz + 2 * qw * qy)
    row_2 = ca.horzcat(2 * qx * qy + 2 * qw * qz, 1 - 2 * qx**2 - 2 * qz**2, 2 * qy * qz - 2 * qw * qx)
    row_3 = ca.horzcat(2 * qx * qz - 2 * qw * qy, 2 * qy * qz + 2 * qw * qx, 1 - 2 * qx**2 - 2 * qy**2)
    rot_wb = ca.vertcat(row_1, row_2, row_3)

    # 0. Wrench in Rotor frame
    # If rotor dynamics are modeled, explicitly use thrust state as force.
    # Else use thrust control which is then assumed to be equal to the thrust state at all times.
    if mpc.include_thrust_model:
        ft_r_z = ft_s
    else:
        ft_r_z = ft_c

    #########################################################################
    # NO NOISE IN THIS MODEL!
    #########################################################################

    # Torque in Rotor frame from thrust and coupling factor kq_d_kt
    # Torque generated by each rotor around its z-axis (reaction torque)
    tau_r_z = -dr * ft_r_z * kq_d_kt

    # 1. Apply transformation from Rotor to End-of-arm: rot_er @ ft_r:
    # [1,         0,         0]   [    0]   [                0]
    # [0,  cos(a_i), -sin(a_i)] @ [    0] = [(-sin(a_i))*ft_ri]
    # [0,  sin(a_i),  cos(a_i)]   [ft_ri]   [   cos(a_i)*ft_ri]
    ft_e_y = -sin_a * ft_r_z
    ft_e_z = cos_a * ft_r_z

    tau_e_y = -sin_a * tau_r_z
    tau_e_z = cos_a * tau_r_z

    # 2. Apply transformation from End-of-arm to Body: rot_be @ ft_e:
    # [cos(theta_i), -sin(theta_i), 0]   [                0]   [(-sin(theta_i))*ft_ri*(-sin(a_i))]   [[(-sin(theta_i))*ft_e_y]
    # [sin(theta_i),  cos(theta_i), 0] @ [ft_ri*(-sin(a_i))] = [   cos(theta_i)*ft_ri*(-sin(a_i))] = [    cos(theta_i)*ft_e_y]
    # [        0,          0,       1]   [   ft_ri*cos(a_i)]   [                   ft_ri*cos(a_i)]   [                 ft_e_z]
    #
    # with sin(theta_i) = pi_b[1] / sqrt(pi_b[0]^2 + pi_b[1]^2)
    # and  cos(theta_i) = pi_b[0] / sqrt(pi_b[0]^2 + pi_b[1]^2)
    ft_b_x = -sin_theta * ft_e_y
    ft_b_y = cos_theta * ft_e_y
    ft_b_z = ft_e_z

    tau_b_x = -sin_theta * tau_e_y
    tau_b_y = cos_theta * tau_e_y
    tau_b_z = tau_e_z

    # 4. Add cross product terms: p_b × f_b for each rotor position
    # Incorporate gyroscopic effects
    cross_tau_b_x = p_b[:, 1] * ft_b_z - p_b[:, 2] * ft_b_y  # p_y * f_z - p_z * f_y
    cross_tau_b_y = p_b[:, 2] * ft_b_x - p_b[:, 0] * ft_b_z  # p_z * f_x - p_x * f_z
    cross_tau_b_z = p_b[:, 0] * ft_b_y - p_b[:, 1] * ft_b_x  # p_x * f_y - p_y * f_x

    # 5. Sum over all rotor contributions
    fu_b = ca.vertcat(ca.sum1(ft_b_x), ca.sum1(ft_b_y), ca.sum1(ft_b_z))

    tau_u_b = ca.vertcat(
        ca.sum1(tau_b_x + cross_tau_b_x), ca.sum1(tau_b_y + cross_tau_b_y), ca.sum1(tau_b_z + cross_tau_b_z)
    )

    # 6. Apply transformation from Body to World: rot_wb @ ft_b:
    # [1 - 2*qy^2 - 2*qz^2, 2*qx*qy - 2*qw*qz, 2*qx*qz + 2*qw*qy]
    # [2*qx*qy + 2*qw*qz, 1 - 2*qx^2 - 2*qz^2, 2*qy*qz - 2*qw*qx] @ [fu_b] = [fu_w]
    # [2*qx*qz - 2*qw*qy, 2*qy*qz + 2*qw*qx, 1 - 2*qx^2 - 2*qy^2]
    fu_w = rot_wb @ fu_b

    # Assemble dynamical model
    x_dot = ca.vertcat(
        v,
        fu_w / mass + g_w,
        (-w[0] * q[1] - w[1] * q[2] - w[2] * q[3]) / 2,  # Convert angular velocity to rotation in World frame
        (w[0] * q[0] + w[2] * q[2] - w[1] * q[3]) / 2,
        (w[1] * q[0] - w[2] * q[1] + w[0] * q[3]) / 2,
        (w[2] * q[0] + w[1] * q[1] - w[0] * q[2]) / 2,
        ca.mtimes(I_inv, (-ca.cross(w, ca.mtimes(I, w)) + tau_u_b)),  # Stay in Body frame
    )

    # - Extend model by servo angle first-order dynamics
    if mpc.include_servo_model:
        x_dot = ca.vertcat(x_dot, (a_c - a_s) / t_servo)  # Time constant of servo motor

    # - Extend model by thrust first-order dynamics
    # Assumption if not included: f_tc = f_ts
    if mpc.include_thrust_model:
        x_dot = ca.vertcat(x_dot, (ft_c - ft_s) / t_rotor)  # Time constant of rotor

    return ca.Function("x_dot", [state, u], [x_dot], ["x", "u"], ["x_dot"])
