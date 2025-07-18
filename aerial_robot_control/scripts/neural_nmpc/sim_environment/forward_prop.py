import numpy as np
import casadi as ca

import os, sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from utils.geometry_utils import skew_symmetric

def init_forward_prop(nmpc):
    p = ca.SX.sym("p", 3)  # Position
    v = ca.SX.sym("v", 3)  # Linear velocity in World frame (inertial reference for Newton's laws)
    q = ca.SX.sym("q", 4)  # Quaternion (representing orientation of the Body frame relative to the World frame)
    w = ca.SX.sym("w", 3)  # Angular velocity in Body frame (principal axes, diagonal inertia matrix)
    state = ca.vertcat(p, v, q, w)

    # - Extend state-space by dynamics of servo angles (actual)
    # Differentiate between actual angles and control angles
    # Note: If servo angle is not used as control input the model for omnidirectional Quadrotor
    # has been observed to be unstable (see https://arxiv.org/abs/2405.09871).
    if nmpc.tilt and nmpc.include_servo_model:
        a_s = ca.SX.sym("a_s", 4)
        state = ca.vertcat(state, a_s)

    # - Extend state-space by dynamics of rotor (actual)
    # Differentiate between actual thrust and control thrust
    if nmpc.include_thrust_model:
        ft_s = ca.SX.sym("ft_s", 4)
        state = ca.vertcat(state, ft_s)

    ft_c = ca.SX.sym("ft_c", 4)
    a_c = ca.SX.sym("a_c", 4)
    u = ca.vertcat(ft_c, a_c)

    # Generate symbolic CasADi function
    dynamics = full_dynamics(nmpc, state, u)
    return dynamics, state, u

def forward_prop(dynamics, state_sym, u_sym, state_0, u_cmd,
                 T_horizon, T_step, m_int_steps=1):
    """
    Propagates forward the state estimate described by the mean vector state_0 and the covariance matrix covar, and a
    sequence of inputs for the system u_seq. These inputs can either be numerical or symbolic.
    :param state_0: Initial nx state vector
    :param u_cmd: Optimized N x nu sequence of control inputs from MPC, with the vector format:
    [u_1(1), u_2(1), u_3(1), u_4(1), ..., u_3(m), u_4(m)]
    :param cov_0: Initial covariance state estimate (default 0). Can be either a positive semi-definite matrix or a
    1D vector, which will be the diagonal of the covariance matrix. In both cases, the resulting covariance matrix
    must be nxn shape, where n is the length of state_0.
    :param T_horizon: time span of the control inputs (default is the time horizon of the MPC)
    :param m_int_steps: Number of integration steps per control input. Default is 1, i.e. the control inputs are
    applied at the end of the integration step.
    :param dt: Optional. Vector of length m, with the corresponding integration time for every control input in
    u_cmd. If none is provided, the default integration step is used.
    :param use_gp: Boolean, whether to use GP regressors when performing the integration or not.
    :param use_model: Integer. Select which dynamics model to use from the available options.
    :return: An m x n array of propagated (expected) state estimates, and an m x n x n array with the m propagated
    covariance matrices.
    """
    if T_horizon % T_step != 0:
        print(f"Warning: T_horizon {T_horizon} is not a multiple of T_step {T_step}.")
    
    # Initialize parameters
    # N = u_cmd.shape[0]    # Number of steps in the integration horizon
    # T_step = T_horizon / N
    # nx = state_0.shape[0]
    N = int(T_horizon / T_step)

    # if cov_0 is None:
    #     cov_0 = np.zeros((nx, nx))
    # elif cov_0.shape == (nx,):
    #     cov_0 = np.diag(cov_0)
    # elif cov_0.shape == (nx, nx):
    #     pass
    # else:
    #     raise ValueError("Invalid shape for covariance matrix")

    # Initialize sequence of propagated states
    state_prop = state_0     # Mean of the state estimate
    # cov_prop = [cov_0]      # Covariance of the state estimate
    # cost_prop = [0.0]       # Cost associated with the state estimate

    # Assume constant input over the time horizon
    u_cmd = np.tile(u_cmd, (N, 1))  # Repeat the control input for each time step

    # Compute forward propagation of state pdf
    # return uncertainty_forward_propagation(state_0, u_cmd, T_horizon, covar=cov_0,
    #                                         discrete_dynamics_f=quad_opt.discretize_f_and_q,
    #                                         dynamics_jac_f=quad_opt.quad_xdot_jac,
    #                                         B_x=B_x, m_integration_steps=1)

    # state_prop, cov_prop, _ = _forward_prop_core(state_0, u_cmd, T_horizon, discrete_dynamics_f, dynamics_jac_f, B_x,
    #                                     gp_regressors, cov_0, dt, m_integration_steps, use_model)

    for k in range(N):
        # Get current control input and current state mean and covariance
        u_k = u_cmd[k, :]
        state_k = state_prop[k, :]
        # cov_k = cov_prop[k]

        # Propagate estimate
        # state(k+1) vector from propagation equations. Pass state through nominal dynamics.
        f = discretize_dynamics_and_cost(dynamics, state_sym, u_sym, T_step, m_int_steps)    # Create casadi function

        f_k = f(state_k, u_k) # Call casadi function with current state and control input
        state_next = np.array(f_k)#['x_f']
        # stage_cost = f_k['qf']

        # K(k+1) matrix from propagation equations
        # K_mat = cov_k

        # Evaluate linearized dynamics at current state
        # l_mat = linearized_dynamics(state_k, u_k) * dt + np.eye(nx)

        # Add next state estimate to lists
        state_prop = np.append(state_prop, state_next.T, axis=0)
        # cov_prop.append(ca.mtimes(ca.mtimes(l_mat, K_mat), l_mat.T))
        # cost_prop.append(stage_cost)

    # cov_prop = cov_prop.reshape((state_prop.shape[1], state_prop.shape[1], -1), order='F').transpose(2, 0, 1)
    return state_prop


def discretize_dynamics_and_cost(dynamics, x, u, T_horizon, m_steps_per_point):
    """
    Integrates the symbolic dynamics and cost equations until the time horizon using a RK4 method.
    :param T_horizon: Time horizon in seconds
    :param N: Number of control input points until time horizon
    :param m_steps_per_point: Number of integrations steps per control input
    :param state: Symbolic vector for state, with length nx
    :param u: Symbolic vector for control input, with length nu
    :param cost_f: symbolic cost function written in CasADi symbolic syntax. If None, then cost 0 is returned.
    :param ind: Only used for trajectory tracking. Index of cost function to use.
    :return: a symbolic function that computes the dynamics integration and the cost function at n_control_inputs
    points until the time horizon given an initial state and
    """


    # if isinstance(cost_f, list):
    #     # Select the list of cost functions
    #     cost_f = cost_f[ind * m_steps_per_point:(ind + 1) * m_steps_per_point]
    # else:
    #     cost_f = [cost_f] * m_steps_per_point

    dt = T_horizon / m_steps_per_point
    # q = 0
    x0 = x  # Initial state

    # Fixed step Runge-Kutta 4 integrator
    for j in range(m_steps_per_point):
        k1 = dynamics(x=x, u=u)['x_dot']
        k2 = dynamics(x=x + dt / 2 * k1, u=u)['x_dot']
        k3 = dynamics(x=x + dt / 2 * k2, u=u)['x_dot']
        k4 = dynamics(x=x + dt * k3, u=u)['x_dot']
        x += dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)

        # if cost_f and cost_f[j] is not None:
        #     q += cost_f[j](x=x, u=u)['q']

    return ca.Function('F', [x0, u], [x], ['x0', 'u'], ['xf'])

def linearized_dynamics(state, input, nx, nu, mass, I):
    """
    Jacobian J matrix of the linearized dynamics specified in the function quad_dynamics. J[i, j] corresponds to
    the partial derivative of f_i(x) wrt x(j).

    :return: a CasADi symbolic function that calculates the nx x nx Jacobian matrix of the linearized simplified
    quadrotor dynamics
    """
    if isinstance(state, ca.SX):
        cs_type = ca.SX
    elif isinstance(state, ca.SX):
        cs_type = ca.SX

    # TODO state is the casadi symbolic state vector from the NMPC object
    p = state[0:3]    # Position
    v = state[3:6]    # Linear velocity
    q = state[6:10]   # Quaternion
    w = state[10:13]  # Angular velocity

    u = input

    jac = cs_type(nx, nx)

    # Position derivatives
    jac[0:3, 3:6] = ca.diag(cs_type.ones(3))

    # Quaternion derivatives
    jac[6:10, 6:10] = skew_symmetric(w) / 2
    jac[6, 10:13] = 1 / 2 * ca.horzcat(-q[1], -q[2], -q[3])
    jac[7, 10:13] = 1 / 2 * ca.horzcat( q[0], -q[3],  q[2])
    jac[8, 10:13] = 1 / 2 * ca.horzcat( q[3],  q[0], -q[1])
    jac[9, 10:13] = 1 / 2 * ca.horzcat(-q[2],  q[1],  q[0])

    # Velocity derivatives
    a_u = (u[0] + u[1] + u[2] + u[3]) / mass
    jac[3, 6:10] = 2 * ca.horzcat( a_u * q[2],      a_u * q[3],      a_u * q[0], a_u * q[1])
    jac[4, 6:10] = 2 * ca.horzcat(-a_u * q[1],     -a_u * q[0],      a_u * q[3], a_u * q[2])
    jac[5, 6:10] = 2 * ca.horzcat(          0, -2 * a_u * q[1], -2 * a_u * q[1],          0)

    # Rate derivatives
    jac[10, 10:13] = (I[1] - I[2]) / I[0] * ca.horzcat(0, w[2], w[1])
    jac[11, 10:13] = (I[2] - I[0]) / I[1] * ca.horzcat(w[2], 0, w[0])
    jac[12, 10:13] = (I[0] - I[1]) / I[2] * ca.horzcat(w[1], w[0], 0)

    return ca.Function('J', [state, u], [jac])


def full_dynamics(nmpc, state, u):
    """
    Symbolic dynamics of the 3D quadrotor model. The state consists on: [p_xyz, a_wxyz, v_xyz, r_xyz]^T, where p
    stands for position, a for angle (in quaternion form), v for velocity and r for body rate. The input of the
    system is: [u_1, u_2, u_3, u_4], i.e. the activation of the four thrusters.

    :param rdrv_d: a 3x3 diagonal matrix containing the D matrix coefficients for a linear drag model as proposed
    by Faessler et al.

    :return: CasADi function that computes the analytical differential state dynamics of the quadrotor model.
    Inputs: 'x' state of quadrotor (6x1) and 'u' control input (2x1). Output: differential state vector 'x_dot'
    (6x1)
    """
    p = state[0:3]
    v = state[3:6]
    q = state[6:10]
    qx = state[6]
    qy = state[7]
    qz = state[8]
    qw = state[9]
    w = state[10:13]
    if nmpc.tilt and nmpc.include_servo_model:
        a_s = state[13:17]
    if nmpc.include_thrust_model:
        ft_s = state[17:21]

    ft_c = u[:4]
    a_c = u[4:8]

    # Physical parameters
    mass = nmpc.phys.mass
    [Ixx, Iyy, Izz] = [nmpc.phys.Ixx, nmpc.phys.Iyy, nmpc.phys.Izz]
    I = ca.diag(ca.vertcat(Ixx, Iyy, Izz))
    I_inv = ca.diag(ca.vertcat(1 / Ixx, 1 / Iyy, 1 / Izz))
    g_w = ca.vertcat(0, 0, -nmpc.phys.gravity)

    t_rotor = nmpc.phys.t_rotor  # Time constant of rotor dynamics
    t_servo = nmpc.phys.t_servo  # Time constant of servo dynamics

    p_b = ca.horzcat(
        nmpc.phys.p1_b,
        nmpc.phys.p2_b,
        nmpc.phys.p3_b,
        nmpc.phys.p4_b
    ).T

    dr = ca.vertcat(
        nmpc.phys.dr1,
        nmpc.phys.dr2,
        nmpc.phys.dr3,
        nmpc.phys.dr4
    )
    kq_d_kt = nmpc.phys.kq_d_kt

    # - Rotor to End-of-arm
    if nmpc.include_servo_model:
        a = a_s
    else:
        a = a_c
    cos_a = ca.cos(a)
    sin_a = ca.sin(a)

    # - End-of-arm to Body
    # Vectorized rotation matrices rot_be (3x3x4)
    # We'll compute the rotated forces later directly without storing the full rotation matrices
    norm_xy = ca.sqrt(p_b[:, 0] ** 2 + p_b[:, 1] ** 2)    # Avoid using norm_2()
    sin_theta = p_b[:, 1] / norm_xy
    cos_theta = p_b[:, 0] / norm_xy

    # - Body to World
    row_1 = ca.horzcat(
        1 - 2 * qy ** 2 - 2 * qz ** 2,
        2 * qx * qy - 2 * qw * qz,
        2 * qx * qz + 2 * qw * qy
    )
    row_2 = ca.horzcat(
        2 * qx * qy + 2 * qw * qz,
        1 - 2 * qx ** 2 - 2 * qz ** 2,
        2 * qy * qz - 2 * qw * qx
    )
    row_3 = ca.horzcat(
        2 * qx * qz - 2 * qw * qy,
        2 * qy * qz + 2 * qw * qx,
        1 - 2 * qx ** 2 - 2 * qy ** 2
    )
    rot_wb = ca.vertcat(row_1, row_2, row_3)

    # 0. Wrench in Rotor frame
    # If rotor dynamics are modeled, explicitly use thrust state as force.
    # Else use thrust control which is then assumed to be equal to the thrust state at all times.
    if nmpc.include_thrust_model:
        ft_r_z = ft_s
    else:
        ft_r_z = ft_c

    #########################################################################
    # NO NOISE IN THIS MODEL!
    #########################################################################

    # Torque in Rotor frame from thrust and coupling factor kq_d_kt
    # Torque generated by each rotor around its z-axis (reaction torque)
    tau_r_z = - dr * ft_r_z * kq_d_kt

    # 1. Apply transformation from Rotor to End-of-arm: rot_er @ ft_r:
    # [1,         0,         0]   [    0]   [                0]
    # [0,  cos(a_i), -sin(a_i)] @ [    0] = [(-sin(a_i))*ft_ri]
    # [0,  sin(a_i),  cos(a_i)]   [ft_ri]   [   cos(a_i)*ft_ri]

    ft_e_y = - sin_a * ft_r_z
    ft_e_z =   cos_a * ft_r_z

    tau_e_y = - sin_a * tau_r_z
    tau_e_z =   cos_a * tau_r_z

    # 2. Apply transformation from End-of-arm to Body: rot_be @ ft_e:
    # [cos(theta_i), -sin(theta_i), 0]   [                0]   [(-sin(theta_i))*ft_ri*(-sin(a_i))]   [[(-sin(theta_i))*ft_e_y]
    # [sin(theta_i),  cos(theta_i), 0] @ [ft_ri*(-sin(a_i))] = [   cos(theta_i)*ft_ri*(-sin(a_i))] = [    cos(theta_i)*ft_e_y]
    # [        0,          0,       1]   [   ft_ri*cos(a_i)]   [                   ft_ri*cos(a_i)]   [                 ft_e_z]
    #
    # with sin(theta_i) = pi_b[1] / sqrt(pi_b[0]^2 + pi_b[1]^2)
    # and  cos(theta_i) = pi_b[0] / sqrt(pi_b[0]^2 + pi_b[1]^2)

    ft_b_x = - sin_theta * ft_e_y
    ft_b_y =   cos_theta * ft_e_y
    ft_b_z =   ft_e_z

    tau_b_x = - sin_theta * tau_e_y
    tau_b_y =   cos_theta * tau_e_y
    tau_b_z =   tau_e_z

    # 4. Add cross product terms: p_b × f_b for each rotor position
    # TODO what do they represent?
    cross_tau_b_x = p_b[:, 1] * ft_b_z - p_b[:, 2] * ft_b_y  # p_y * f_z - p_z * f_y
    cross_tau_b_y = p_b[:, 2] * ft_b_x - p_b[:, 0] * ft_b_z  # p_z * f_x - p_x * f_z
    cross_tau_b_z = p_b[:, 0] * ft_b_y - p_b[:, 1] * ft_b_x  # p_x * f_y - p_y * f_x

    # 5. Sum over all rotor contributions
    fu_b = ca.vertcat(
        ca.sum1(ft_b_x),
        ca.sum1(ft_b_y),
        ca.sum1(ft_b_z)
    )

    tau_u_b = ca.vertcat(
        ca.sum1(tau_b_x + cross_tau_b_x),
        ca.sum1(tau_b_y + cross_tau_b_y),
        ca.sum1(tau_b_z + cross_tau_b_z)
    )

    # 6. Apply transformation from Body to World: rot_wb @ ft_b:
    # [1 - 2*qy^2 - 2*qz^2, 2*qx*qy - 2*qw*qz, 2*qx*qz + 2*qw*qy]
    # [2*qx*qy + 2*qw*qz, 1 - 2*qx^2 - 2*qz^2, 2*qy*qz - 2*qw*qx] @ [fu_b] = [fu_w]
    # [2*qx*qz - 2*qw*qy, 2*qy*qz + 2*qw*qx, 1 - 2*qx^2 - 2*qy^2]

    fu_w = rot_wb @ fu_b

    # Assemble dynamical model
    x_dot = ca.vertcat(
        p_dynamics(v),
        v_dynamics(fu_w, mass, g_w),
        q_dynamics(q, w),
        w_dynamics(tau_u_b, w, I, I_inv)
    )
    if nmpc.include_servo_model and not nmpc.include_servo_derivative:
        x_dot = ca.vertcat(x_dot,
                          (a_c - a_s) / t_servo  # Time constant of servo motor
                          )

    # - Extend model by thrust first-order dynamics
    # Assumption if not included: f_tc = f_ts
    if nmpc.include_thrust_model:
        x_dot = ca.vertcat(x_dot,
                          (ft_c - ft_s) / t_rotor  # Time constant of rotor
                          )

    if nmpc.include_cog_dist_model:
        x_dot = ca.vertcat(x_dot,
                        ca.vertcat(0.0, 0.0, 0.0),
                        ca.vertcat(0.0, 0.0, 0.0),
                        )
            
    return ca.Function('x_dot', [state, u], [x_dot], ['x', 'u'], ['x_dot'])

def p_dynamics(v):
    return v

def v_dynamics(fu_w, mass, g_w):
    """
    TODO implement drag compensation rdrv_d
    :param rdrv_d: a 3x3 diagonal matrix containing the D matrix coefficients for a linear drag model as proposed
    by Faessler et al. None, if no linear compensation is to be used.
    """
    return fu_w / mass + g_w

def q_dynamics(q, w):
    return ca.vertcat(
    (-w[0] * q[1] - w[1] * q[2] - w[2] * q[3]) / 2,   # Convert angular velocity to rotation in World frame
    ( w[0] * q[0] + w[2] * q[2] - w[1] * q[3]) / 2,
    ( w[1] * q[0] - w[2] * q[1] + w[0] * q[3]) / 2,
    ( w[2] * q[0] + w[1] * q[1] - w[0] * q[2]) / 2)

def w_dynamics(tau_u_b, w, I, I_inv):
    return ca.mtimes(I_inv, (-ca.cross(w, ca.mtimes(I, w)) + tau_u_b))