import os, sys
import numpy as np
import torch
import casadi as ca
from acados_template import AcadosModel, AcadosOcpSolver

from utils.data_utils import delete_previous_solver_files
from utils.geometry_utils import quaternion_inverse, v_dot_q
from utils.model_utils import load_model, get_output_mapping
from utils.model_utils import cross_check_params

# Tiltable-Quadrotor
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from nmpc.nmpc_tilt_mt.rh_base import RecedingHorizonBase
from nmpc.nmpc_tilt_mt.tilt_qd.qd_reference_generator import QDNMPCReferenceGenerator
import nmpc.nmpc_tilt_mt.tilt_qd.phys_param_beetle_omni as phys_omni


class NeuralMPC(RecedingHorizonBase):
    def __init__(self, model_options, solver_options, sim_options, run_options, use_as_simulator=False):
        # TODO implement solver options flexibly such as which solver to use, etc.
        # TODO implement drag correction with RDRv (?)
        if model_options["mpc_type"] != "NMPCTiltQdServo":
            raise NotImplementedError(
                "Model flags, Model name, Parameter loading, Weights, Cost function and Reference generation "
                "are only implemented for NMPCTiltQdServo."
            )

        # Model name and type
        self.model_options = model_options
        self.arch_type = model_options["arch_type"]
        self.mpc_type = model_options["mpc_type"]
        if model_options["only_use_nominal"]:
            identifier = "nominal"
            identifier2 = ""
        else:
            identifier = "neural"
            identifier2 = ""
            if "minus_neural" in model_options:
                if model_options["minus_neural"]:
                    identifier2 = "_minus"

            if "plus_neural" in model_options:
                if model_options["plus_neural"]:
                    identifier2 = "_plus"

        if use_as_simulator:
            identifier3 = "_sim"
        else:
            identifier3 = ""

        self.model_name = f"tilt_qd_{identifier}_servo{identifier2}{identifier3}_mdl"

        # Read controller parameters from configuration file in the robot's package
        self.read_params("controller", "nmpc", "beetle_omni", "BeetleNMPCFull.yaml")
        self.T_samp = self.params["T_samp"]  # Sampling time for the MPC controller, time step between two steps
        self.T_horizon = self.params["T_horizon"]  # Time horizon for optimization loop in MPC controller
        self.N = self.params["N_steps"]  # Number of MPC nodes
        self.T_step = self.params["T_step"]  # Step size used in optimization loop in MPC controller

        # Store physical parameters
        self.phys = phys_omni

        # Define model flags based on architecture type
        if self.arch_type == "tilt_qd":
            self.tilt = True
            if self.mpc_type == "NMPCTiltQdServo":
                self.include_servo_model = True
                self.include_thrust_model = False
                self.include_cog_dist_model = False
                self.include_impedance = False

        # Include disturbance parameters in model
        # NOTE: ONLY FOR SIMULATOR USAGE
        self.include_cog_dist_parameter = sim_options["disturbances"]["cog_dist"]
        self.include_motor_noise_parameter = sim_options["disturbances"]["motor_noise"]

        # Solver options
        self.include_floor_bounds = solver_options["include_floor_bounds"]
        self.include_soft_constraints = solver_options["include_soft_constraints"]
        self.include_quaternion_constraint = solver_options["include_quaternion_constraint"]

        # Load neural network model
        if not model_options["only_use_nominal"]:
            self.use_mlp = True
            # Load pre-trained MLP
            self.neural_model, self.mlp_metadata = load_model(model_options, sim_options, run_options)
            # Cross-check meta parameters used for MPC to train MLP
            cross_check_params(self.params, self.mlp_metadata)
            print(
                f"Successfully loaded MLP model {model_options['neural_model_name']}, {model_options['neural_model_instance']}."
            )
        else:
            self.use_mlp = False

        # Clean up so the build process doesn't depend on previously generated files
        delete_previous_solver_files(self.model_name)

        # Call RecedingHorizon constructor to create the acados model, OCP, and solver objects
        super().__init__(method="neural_mpc", build=True)

        # Create Reference Generator object
        self._reference_generator = self._create_reference_generator()

        # Store acados model properties
        self.state = self._acados_model.x
        self.controls = self._acados_model.u
        self.parameters = self._acados_model.p

    # fmt: off
    def create_acados_model(self) -> AcadosModel:
        """
        Define generic state-space, acados model parameters, control inputs for kinematics of a omnidirectional quadrotor.
        Calculate transformation matrix from robot's architecture to compute internal wrench.
        Assemble acados model based on given cost function.
        """
        # Standard state-space (Note: store in self to access for cost function in child controller class)
        self.p = ca.MX.sym("p", 3)  # Position
        self.v = ca.MX.sym("v", 3)  # Linear velocity in World frame (inertial reference for Newton's laws)
        self.q = ca.MX.sym("q", 4)  # Quaternion (representing orientation of the Body frame relative to the World frame)
        self.qw = self.q[0]
        self.qx = self.q[1]
        self.qy = self.q[2]
        self.qz = self.q[3]
        self.w = ca.MX.sym("w", 3)  # Angular velocity in Body frame (principal axes, diagonal inertia matrix)
        self.wx = self.w[0]
        self.wy = self.w[1]
        self.wz = self.w[2]
        state = ca.vertcat(self.p, self.v, self.q, self.w)

        # - Extend state-space by dynamics of servo angles (actual)
        # Differentiate between actual angles and control angles
        # NOTE: Including the servo angle as state and therefore incorporating the servo dynamics into the model for an omnidirectional quadrotor
        # has been shown to improve performance (see Li et al. https://arxiv.org/abs/2405.09871).
        if self.tilt and self.include_servo_model:
            self.a_s = ca.MX.sym("a_s", 4)
            state = ca.vertcat(state, self.a_s)

        # - Extend state-space by dynamics of rotor (actual)
        # Differentiate between actual thrust and control thrust
        if self.include_thrust_model:
            self.ft_s = ca.MX.sym("ft_s", 4)
            state = ca.vertcat(state, self.ft_s)

        # - Extend state-space by disturbance on CoG (actual)
        # Differentiate between actual disturbance set as state and set as parameter
        if self.include_cog_dist_model:
            # Force disturbance applied to CoG in World frame
            self.fds_w = ca.MX.sym("fds_w", 3)
            # Torque disturbance applied to CoG in Body frame
            self.tau_ds_b = ca.MX.sym("tau_ds_b", 3)

            state = ca.vertcat(state, self.fds_w, self.tau_ds_b)
        else:
            self.fds_w = ca.MX.zeros(3)
            self.tau_ds_b = ca.MX.zeros(3)

        # Control inputs
        # - Forces from thrust at each rotor
        self.ft_c = ca.MX.sym("ft_c", 4)
        controls = ca.vertcat(self.ft_c)
        # - Servo angle for tiltable rotors (actuated)
        if self.tilt:
            self.a_c = ca.MX.sym("a_c", 4)
            controls = ca.vertcat(controls, self.a_c)

        # Model parameters
        self.qwr = ca.MX.sym("qwr")     # Reference for quaternions
        self.qxr = ca.MX.sym("qxr")
        self.qyr = ca.MX.sym("qyr")
        self.qzr = ca.MX.sym("qzr")
        parameters = ca.vertcat(self.qwr, self.qxr, self.qyr, self.qzr)

        # Make physical parameters available in the model
        mass = ca.MX.sym("mass")
        gravity = ca.MX.sym("gravity")

        Ixx = ca.MX.sym("Ixx")
        Iyy = ca.MX.sym("Iyy")
        Izz = ca.MX.sym("Izz")

        kq_d_kt = ca.MX.sym("kq_d_kt")  # Coupling factor between thrust and torque

        dr1 = ca.MX.sym("dr1")          # Distance from CoG to rotors
        dr2 = ca.MX.sym("dr2")
        dr3 = ca.MX.sym("dr3")
        dr4 = ca.MX.sym("dr4")
        dr = ca.vertcat(dr1, dr2, dr3, dr4)

        p1_b = ca.MX.sym("p1_b", 3)     # Position of rotors in Body frame
        p2_b = ca.MX.sym("p2_b", 3)
        p3_b = ca.MX.sym("p3_b", 3)
        p4_b = ca.MX.sym("p4_b", 3)
        p_b = ca.horzcat(p1_b, p2_b, p3_b, p4_b).T

        t_rotor = ca.MX.sym("t_rotor")  # Time constant of thrust dynamics
        t_servo = ca.MX.sym("t_servo")  # Time constant of servo dynamics

        # Position and quaternion of an end-effector in CoG frame
        self.ee_p = ca.MX.sym("ee_p", 3)
        self.ee_q = ca.MX.sym("ee_qwxyz", 4)  # qw, qx, qy, qz

        phy_params = ca.vertcat(
            mass, gravity, Ixx, Iyy, Izz, kq_d_kt,
            dr1, p1_b, dr2, p2_b, dr3, p3_b, dr4, p4_b, t_rotor, t_servo,
            self.ee_p, self.ee_q
        )
        parameters = ca.vertcat(parameters, phy_params)

        # - Extend model parameters by CoG disturbance
        if self.include_cog_dist_parameter:
            # Force disturbance applied to CoG in World frame
            self.fdp_w = ca.MX.sym("fdp_w", 3)
            # Torque disturbance applied to CoG in Body frame
            self.tau_dp_b = ca.MX.sym("tau_dp_b", 3)

            self.cog_dist_start_idx = parameters.size()[0]
            parameters = ca.vertcat(parameters, self.fdp_w, self.tau_dp_b)
            self.cog_dist_end_idx = parameters.size()[0]
        else:
            self.fdp_w = ca.MX.zeros(3)
            self.tau_dp_b = ca.MX.zeros(3)

        # - Extend model parameters by virtual mass and inertia for impedance cost function
        if self.include_impedance:
            if not self.include_cog_dist_model or not self.include_cog_dist_parameter:
                raise ValueError("Impedance cost can only be calculated if disturbance flags are activated.")
            # TODO Optimize such that less separate variables are created
            mpx = ca.MX.sym("mpx")  # Virtual mass (p = position)
            mpy = ca.MX.sym("mpy")
            mpz = ca.MX.sym("mpz")
            self.mp = ca.vertcat(mpx, mpy, mpz)

            mqx = ca.MX.sym("mqx")  # Virtual inertia (q = quaternion)
            mqy = ca.MX.sym("mqy")
            mqz = ca.MX.sym("mqz")
            self.mq = ca.vertcat(mqx, mqy, mqz)

            parameters = ca.vertcat(parameters, self.mp, self.mq)

        # - Extend model parameters by noise on the nominal thrust of each rotor in Body frame
        if self.include_motor_noise_parameter:
            self.fnp_b = ca.MX.sym("fnp_b", 4)
            if self.tilt:
                self.anp_b = ca.MX.sym("anp_b", 4)
            else:
                self.anp_b = ca.MX.sym("anp_b", 0)

            self.motor_noise_start_idx = parameters.size()[0]
            parameters = ca.vertcat(parameters, self.fnp_b, self.anp_b)
            self.motor_noise_end_idx = parameters.size()[0]
        else:
            self.fnp_b = ca.MX.zeros(4)
            self.anp_b = ca.MX.zeros(4)

        # Transformation between coordinate systems World, Body, End-of-arm, Rotor using quaternions
        # - Rotor to End-of-arm
        # Take tilt rotation with angle alpha (a) of R frame to E frame into account
        if self.tilt:
            # If servo dynamics are modeled, use angle state.
            # Else use angle control which is then assumed to be equal to the angle state at all times.
            if self.include_servo_model:
                a = self.a_s
            else:
                a = self.a_c

            # Add motor noise if specified
            a += self.anp_b

            # Compute cos and sin of angles
            cos_a = ca.cos(a)
            sin_a = ca.sin(a)
        else:
            cos_a = ca.MX.ones(4,1)
            sin_a = ca.MX.zeros(4,1)

        # - End-of-arm to Body
        # Vectorized rotation matrices rot_be (3x3x4)
        # We'll compute the rotated forces later directly without storing the full rotation matrices
        norm_xy = ca.sqrt(p_b[:, 0] ** 2 + p_b[:, 1] ** 2)    # Avoid using norm_2()
        sin_theta = p_b[:, 1] / norm_xy
        cos_theta = p_b[:, 0] / norm_xy

        # - Body to World
        row_1 = ca.horzcat(
            1 - 2 * self.qy ** 2 - 2 * self.qz ** 2,
            2 * self.qx * self.qy - 2 * self.qw * self.qz,
            2 * self.qx * self.qz + 2 * self.qw * self.qy
        )
        row_2 = ca.horzcat(
            2 * self.qx * self.qy + 2 * self.qw * self.qz,
            1 - 2 * self.qx ** 2 - 2 * self.qz ** 2,
            2 * self.qy * self.qz - 2 * self.qw * self.qx
        )
        row_3 = ca.horzcat(
            2 * self.qx * self.qz - 2 * self.qw * self.qy,
            2 * self.qy * self.qz + 2 * self.qw * self.qx,
            1 - 2 * self.qx ** 2 - 2 * self.qy ** 2
        )
        rot_wb = ca.vertcat(row_1, row_2, row_3)

        # 0. Wrench in Rotor frame
        # If rotor dynamics are modeled, explicitly use thrust state as force.
        # Else use thrust control which is then assumed to be equal to the thrust state at all times.
        if self.include_thrust_model:
            ft_r_z = self.ft_s
        else:
            ft_r_z = self.ft_c

        # Add motor noise if specified
        ft_r_z += self.fnp_b

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
        # Note: The torque is in Body frame in the dynamics to make computation easier.

        # Compute Inertia
        I = ca.diag(ca.vertcat(Ixx, Iyy, Izz))
        I_inv = ca.diag(ca.vertcat(1 / Ixx, 1 / Iyy, 1 / Izz))
        g_w = ca.vertcat(0, 0, -gravity)  # World frame

        # Dynamic model (Time-derivative of state)
        ds = ca.vertcat(
            self.v,
            (fu_w + self.fds_w + self.fdp_w) / mass + g_w,
            (-self.wx * self.qx - self.wy * self.qy - self.wz * self.qz) / 2,  # Convert angular velocity to rotation in World frame
            ( self.wx * self.qw + self.wz * self.qy - self.wy * self.qz) / 2,
            ( self.wy * self.qw - self.wz * self.qx + self.wx * self.qz) / 2,
            ( self.wz * self.qw + self.wy * self.qx - self.wx * self.qy) / 2,
            ca.mtimes(I_inv, (-ca.cross(self.w, ca.mtimes(I, self.w)) + tau_u_b + self.tau_ds_b + self.tau_dp_b)),  # Stay in Body frame
        )

        # - Extend model by servo first-order dynamics
        # Assumption if not included: a_c = a_s
        if self.include_servo_model:
            ds = ca.vertcat(ds,
                (self.a_c - self.a_s) / t_servo  # Time constant of servo motor
            )

        # - Extend model by thrust first-order dynamics
        # Assumption if not included: f_tc = f_ts
        if self.include_thrust_model:
            ds = ca.vertcat(ds,
                (self.ft_c - self.ft_s) / t_rotor  # Time constant of rotor
            )

        # - Extend model by disturbances simply to match state dimensions
        if self.include_cog_dist_model:
            ds = ca.vertcat(ds,
                ca.MX.zeros(3),
                ca.MX.zeros(3),
            )

        # Neural network model: Add predicted residual dynamics to nominal dynamics
        if self.use_mlp:
            self.state_feats = eval(self.mlp_metadata["ModelFitConfig"]["state_feats"])
            self.u_feats = eval(self.mlp_metadata["ModelFitConfig"]["u_feats"])
            self.y_reg_dims = np.array(eval(self.mlp_metadata["ModelFitConfig"]["y_reg_dims"]))
            input_transform = self.mlp_metadata["ModelFitConfig"]["input_transform"]
            label_transform = self.mlp_metadata["ModelFitConfig"]["label_transform"]
            if np.array_equal(self.y_reg_dims, np.array([5])):
                # Map single output to all acceleration dimensions
                only_vz = True
            else:
                only_vz = False

            # === MLP input ===
            if input_transform:
                # MLP is trained to receive and predict the velocity in the Body frame
                # Transform input velocity to body frame
                v_b = v_dot_q(state[3:6], quaternion_inverse(state[6:10]))
                state_in = ca.vertcat(state[:3], v_b, state[6:])
            else:
                state_in = state

            # Assemble MLP input from selected state and control features
            if self.mlp_metadata["ModelFitConfig"]["control_averaging"]:
                if {0, 1, 2, 3}.issubset(self.u_feats):
                    controls_in = ca.sum(controls[0:4]) / 4.0
                if {4, 5, 6, 7}.issubset(self.u_feats):
                    controls_in = ca.vertcat(controls_in, ca.sum(controls[4:8]) / 4.0)
            else:
                controls_in = controls[self.u_feats]
            mlp_in = ca.vertcat(state_in[self.state_feats], controls_in)

            if "temporal" in self.mlp_metadata["MLPConfig"]["model_name"]:
                # Use previous (i.e. delayed) states and controls time steps as input to the MLP
                delay = self.mlp_metadata["MLPConfig"]["delay_horizon"]  # Delay as number of previous time steps
                state_prev, controls_prev = self.append_delay(delay)

                # Append each previous state and control to MLP input
                for i in range(delay):
                    # States are sorted from newest to oldest in acados parameters
                    # Get state components from previous states that are stored concatenated after each other
                    state_prev_i = state_prev[:, i]
                    controls_prev_i = controls_prev[:, i]
                    # Transform input velocity to Body frame
                    v_b = v_dot_q(state_prev_i[3:6], quaternion_inverse(state_prev_i[6:10]))
                    state_prev_i[3:6] = v_b  # Replace velocity in World frame with Body frame velocity

                    mlp_in = ca.vertcat(mlp_in, state_prev_i[self.state_feats], controls_prev_i[self.u_feats])

            # === MLP forward pass ===
            with torch.no_grad():
                if self.model_options["approximate_mlp"]:
                    # TODO investigate this function and parallel Flag!
                    # Parallel flag only active for order == 2
                    mlp_out = self.neural_model.approx(mlp_in, order=self.model_options["approx_order"], parallel=False)
                    approx_params = self.neural_model.sym_approx_params(order=self.model_options["approx_order"], flat=True)
                    self.approx_start_idx = parameters.size()[0]
                    parameters = ca.vertcat(parameters, approx_params)
                    self.approx_end_idx = parameters.size()[0]
                else:
                    mlp_out = self.neural_model(mlp_in)

            if label_transform:
                # Network is trained to predict the velocity in Body frame
                # Transform velocity back to World frame
                if set([3, 4, 5]).issubset(set(self.y_reg_dims)):
                    a_idx = np.where(self.y_reg_dims == 3)[0][0]  # Assumed that v_x, v_y, v_z are consecutive
                    a_b = mlp_out[a_idx : a_idx + 3]
                    a_w = v_dot_q(a_b, state[6:10])
                    mlp_out = ca.vertcat(mlp_out[:a_idx, :], a_w, mlp_out[a_idx + 3 :, :])
                elif set([5]).issubset(set(self.y_reg_dims)):
                    # Predict only a_z so set a_x and a_y to 0 in Body frame and then transform to World frame
                    # The predicted a_z therefore also has influence on the x and y accelerations in World frame
                    # Mapping is later adjusted
                    a_idx = np.where(self.y_reg_dims == 5)[0][0]
                    a_b = ca.vertcat(0, 0, mlp_out[a_idx])
                    a_w = v_dot_q(a_b, state[6:10])
                    mlp_out = ca.vertcat(mlp_out[:a_idx, :], a_w, mlp_out[a_idx + 1 :, :])
                else:
                    raise KeyError("Selected regression dimensions not expected.")

            # Normalize the MLP output by weight of the robot if specified
            # Idea: Model predicts acceleration so to predict the forces we need to scale by mass of the robot
            # TODO doesnt really make sense
            if self.model_options["scale_by_weight"]:
                mlp_out /= self.phys.mass

            # === Fuse dynamics ===
            # Map output of MLP to the state space
            M = get_output_mapping(
                state.shape[0], self.y_reg_dims, label_transform=label_transform, only_vz=only_vz
            )

            # Combine nominal dynamics with neural dynamics
            if self.model_options["minus_neural"]:
                ds -= M @ mlp_out
            elif self.model_options["plus_neural"]:
                ds += M @ mlp_out
            else:
                raise ValueError("Either 'minus_neural' or 'plus_neural' must be set to True in model options when using Neural MPC.")

            # === Time-dependent control law ===
            # Add a symbolic counter to the state vector
            # Assume last state entry is reserved for step_count
            # step_count = ca.MX.sym("step_count", 1)
            # self.state = ca.vertcat(self.state, step_count)
            # Adjust nominal_dynamics and mlp_out to match new state size
            # nominal_dynamics = self.nominal_model.f_expl_expr[:-1]  # exclude counter from nominal dynamics
            # Define switching time in steps (e.g., 2 seconds / T_samp)
            # switch_steps = int(2.0 / self.T_samp)

            # Use CasADi's if_else for switching
            # height_limit = 5.0  # m
            # k = 1.0  # steepness, increase if you want faster switching
            # alpha = 0.5 * (ca.tanh(k * (self.state[2] - height_limit)) + 1)  # ~0 when z<0.1, ~1 when z>0.1
            # f_total = nominal_dynamics + alpha * (M @ mlp_out)
            # f_total = ca.if_else(
            #     height_limit < self.state[2],
            #     nominal_dynamics,
            #     f_total
            # )
        else:
            # Only use nominal dynamics
            pass

        # Assemble acados function
        f = ca.Function("f", [state, controls], [ds], ["state", "control_input"], ["ds"], {"allow_free": True})

        # Implicit dynamics
        # Note: Used only mainly because of acados template
        x_dot = ca.MX.sym("x_dot", state.size())  # Combined state vector
        f_impl = x_dot - f(state, controls)

        # Get terms of cost function
        if self.include_impedance:
            raise NotImplementedError("Impedance cost not implemented for Neural controllers yet.")
            # Compute linear acceleration (in World frame) and angular acceleration (in Body frame) for impedance cost
            # Note: the wrench from I Term and Wrench Est are all important for this term. If we don't consider I Term,
            # a constant disturbance will be injected.
            lin_acc_w = (ca.mtimes(rot_wb, fu_b) + self.fds_w + self.fdp_w) / mass + g_w
            ang_acc_b = ca.mtimes(
                I_inv, (-ca.cross(self.w, ca.mtimes(I, self.w)) + tau_u_b + self.tau_ds_b + self.tau_dp_b)
            )

            state_y, state_y_e, control_y = self.get_cost_function(lin_acc_w=lin_acc_w, ang_acc_b=ang_acc_b)
        else:
            state_y, state_y_e, control_y = self.get_cost_function()

        # Assemble acados model
        model = AcadosModel()
        model.name = self.model_name
        model.f_expl_expr = f(state, controls)  # CasADi expression for the explicit dynamics
        model.f_impl_expr = f_impl              # CasADi expression for the implicit dynamics
        model.x = state
        model.xdot = x_dot
        model.u = controls
        model.p = parameters
        model.cost_y_expr_0 = ca.vertcat(state_y, control_y)  # NONLINEAR_LS
        model.cost_y_expr = ca.vertcat(state_y, control_y)  # NONLINEAR_LS
        model.cost_y_expr_e = state_y_e

        return model

    def get_cost_function(self, lin_acc_w=None, ang_acc_b=None) -> tuple[ca.MX, ca.MX, ca.MX]:
        #### ONLY FOR NMPCServo() FOR NOW ####
        # Cost function
        # see https://docs.acados.org/python_interface/#acados_template.acados_ocp_cost.AcadosOcpCost for details
        # error = y - y_ref
        # NONLINEAR_LS = error^T @ Q @ error
        # qe = qr^* quaternion-multiply q
        qe_x =  self.qwr * self.qx - self.qw * self.qxr - self.qyr * self.qz + self.qy * self.qzr
        qe_y =  self.qwr * self.qy - self.qw * self.qyr + self.qxr * self.qz - self.qx * self.qzr
        qe_z = -self.qxr * self.qy + self.qx * self.qyr + self.qwr * self.qz - self.qw * self.qzr

        state_y = ca.vertcat(
            self.p,
            self.v,
            self.qwr,
            qe_x + self.qxr,
            qe_y + self.qyr,
            qe_z + self.qzr,
            self.w,
            self.a_s,
        )

        state_y_e = state_y

        control_y = ca.vertcat(
            self.ft_c,
            self.a_c - self.a_s  # a_c_ref must be zero to ensure physical consistency for servo angle state!
        )

        return state_y, state_y_e, control_y

    def get_weights(self) -> tuple[np.ndarray, np.ndarray]:
        # Define Weights
        Q = np.diag(
            [
                self.params["Qp_xy"],
                self.params["Qp_xy"],
                self.params["Qp_z"],
                self.params["Qv_xy"],
                self.params["Qv_xy"],
                self.params["Qv_z"],
                0,
                self.params["Qq_xy"],
                self.params["Qq_xy"],
                self.params["Qq_z"],
                self.params["Qw_xy"],
                self.params["Qw_xy"],
                self.params["Qw_z"],
                self.params["Qa"],
                self.params["Qa"],
                self.params["Qa"],
                self.params["Qa"],
            ]
        )
        print("Q: \n", Q)

        R = np.diag(
            [
                self.params["Rt"],
                self.params["Rt"],
                self.params["Rt"],
                self.params["Rt"],
                self.params["Rac_d"],
                self.params["Rac_d"],
                self.params["Rac_d"],
                self.params["Rac_d"],
            ]
        )
        print("R: \n", R)

        return Q, R

    def create_acados_ocp_solver(self, build: bool = True) -> AcadosOcpSolver:
        """
        Create generic acados solver for MPC framework of a quadrotor.
        Generate c code into source folder in aerial_robot_control to be used in workflow.
        """
        # TODO Extend constraints ?
        # TODO Extend cost function ?
        # TODO CHECK SOLVER OPTIONS ?
        # Get OCP object
        ocp = super().get_ocp()

        # Model dimensions
        nn = ocp.solver_options.N_horizon
        nx = ocp.model.x.size()[0]
        nu = ocp.model.u.size()[0]
        n_param = ocp.model.p.size()[0]

        # Get weights from parametrization child file
        Q, R = self.get_weights()

        # Cost function options
        # see https://docs.acados.org/python_interface/#acados_template.acados_ocp_cost.AcadosOcpCost for details
        ocp.cost.cost_type = "NONLINEAR_LS"
        ocp.cost.cost_type_e = "NONLINEAR_LS"
        ocp.cost.W = np.block([[Q, np.zeros((nx, nu))], [np.zeros((nu, nx)), R]])
        ocp.cost.W_e = Q  # Weight matrix at terminal shooting node (N)

        # Set constraints
        # - State box constraints bx
        # -- Index for vx, vy, vz, wx, wy, wz
        ocp.constraints.idxbx = np.array([3, 4, 5, 10, 11, 12])

        # -- Index for a1s, a2s, a3s, a4s
        if self.tilt and self.include_servo_model:
            ocp.constraints.idxbx = np.append(ocp.constraints.idxbx, [13, 14, 15, 16])

            # -- Index for ft1s, ft2s, ft3s, ft4s (When included servo AND thrust, add further indices)
            if self.include_thrust_model:
                ocp.constraints.idxbx = np.append(ocp.constraints.idxbx, [17, 18, 19, 20])

        # -- Index for ft1s, ft2s, ft3s, ft4s (When only included thrust, use the same indices)
        elif self.include_thrust_model:
            ocp.constraints.idxbx = np.append(ocp.constraints.idxbx, [13, 14, 15, 16])

        # -- Index for height z --
        if self.include_floor_bounds:
            ocp.constraints.idxbx = np.append([2], ocp.constraints.idxbx)

        # -- Lower State Bound
        ocp.constraints.lbx = np.array(
            [
                self.params["v_min"],
                self.params["v_min"],
                self.params["v_min"],
                self.params["w_min"],
                self.params["w_min"],
                self.params["w_min"],
            ]
        )

        if self.tilt and self.include_servo_model:
            ocp.constraints.lbx = np.append(
                ocp.constraints.lbx,
                [
                    self.params["a_min"],
                    self.params["a_min"],
                    self.params["a_min"],
                    self.params["a_min"],
                ],
            )

        if self.include_thrust_model:
            ocp.constraints.lbx = np.append(
                ocp.constraints.lbx,
                [
                    self.params["thrust_min"],
                    self.params["thrust_min"],
                    self.params["thrust_min"],
                    self.params["thrust_min"],
                ],
            )

        if self.include_floor_bounds:
            ocp.constraints.lbx = np.append([0], ocp.constraints.lbx)

        # -- Upper State Bound
        ocp.constraints.ubx = np.array(
            [
                self.params["v_max"],
                self.params["v_max"],
                self.params["v_max"],
                self.params["w_max"],
                self.params["w_max"],
                self.params["w_max"],
            ]
        )

        if self.tilt and self.include_servo_model:
            ocp.constraints.ubx = np.append(
                ocp.constraints.ubx,
                [
                    self.params["a_max"],
                    self.params["a_max"],
                    self.params["a_max"],
                    self.params["a_max"]
                ],
            )

        if self.include_thrust_model:
            ocp.constraints.ubx = np.append(
                ocp.constraints.ubx,
                [
                    self.params["thrust_max"],
                    self.params["thrust_max"],
                    self.params["thrust_max"],
                    self.params["thrust_max"],
                ],
            )

        if self.include_floor_bounds:
            ocp.constraints.ubx = np.append(
                [1e8], ocp.constraints.ubx  # TODO there has to be a better way to implement one sided constraint
            )

        # - Terminal state box constraints bx_e
        # -- Index for vx, vy, vz, wx, wy, wz
        ocp.constraints.idxbx_e = np.array([3, 4, 5, 10, 11, 12])

        # -- Index for a1s, a2s, a3s, a4s
        if self.tilt and self.include_servo_model:
            ocp.constraints.idxbx_e = np.append(ocp.constraints.idxbx_e, [13, 14, 15, 16])

            # -- Index for ft1s, ft2s, ft3s, ft4s (When included servo AND thrust, add further indices)
            if self.include_thrust_model:
                ocp.constraints.idxbx_e = np.append(ocp.constraints.idxbx_e, [17, 18, 19, 20])

        # -- Index for ft1s, ft2s, ft3s, ft4s (When only included thrust, use the same indices)
        elif self.include_thrust_model:
            ocp.constraints.idxbx_e = np.append(ocp.constraints.idxbx_e, [13, 14, 15, 16])

        # -- Index for height z --
        if self.include_floor_bounds:
            ocp.constraints.idxbx_e = np.append([2], ocp.constraints.idxbx_e)

        # -- Lower Terminal State Bound
        ocp.constraints.lbx_e = np.array(
            [
                self.params["v_min"],
                self.params["v_min"],
                self.params["v_min"],
                self.params["w_min"],
                self.params["w_min"],
                self.params["w_min"],
            ]
        )

        if self.tilt and self.include_servo_model:
            ocp.constraints.lbx_e = np.append(
                ocp.constraints.lbx_e,
                [
                    self.params["a_min"],
                    self.params["a_min"],
                    self.params["a_min"],
                    self.params["a_min"],
                ],
            )

        if self.include_thrust_model:
            ocp.constraints.lbx_e = np.append(
                ocp.constraints.lbx_e,
                [
                    self.params["thrust_min"],
                    self.params["thrust_min"],
                    self.params["thrust_min"],
                    self.params["thrust_min"],
                ],
            )

        if self.include_floor_bounds:
            ocp.constraints.lbx_e = np.append([0], ocp.constraints.lbx_e)

        # -- Upper Terminal State Bound
        ocp.constraints.ubx_e = np.array(
            [
                self.params["v_max"],
                self.params["v_max"],
                self.params["v_max"],
                self.params["w_max"],
                self.params["w_max"],
                self.params["w_max"],
            ]
        )

        if self.tilt and self.include_servo_model:
            ocp.constraints.ubx_e = np.append(
                ocp.constraints.ubx_e,
                [
                    self.params["a_max"],
                    self.params["a_max"],
                    self.params["a_max"],
                    self.params["a_max"],
                ],
            )

        if self.include_thrust_model:
            ocp.constraints.ubx_e = np.append(
                ocp.constraints.ubx_e,
                [
                    self.params["thrust_max"],
                    self.params["thrust_max"],
                    self.params["thrust_max"],
                    self.params["thrust_max"],
                ],
            )

        if self.include_floor_bounds:
            ocp.constraints.ubx_e = np.append([1e8], ocp.constraints.ubx_e)

        # - Input box constraints bu
        # TODO Potentially a good idea to omit the input constraint when set the equivalent state
        # -- Index for ft1c, ft2c, ft3c, ft4c
        ocp.constraints.idxbu = np.array([0, 1, 2, 3])
        # -- Index for a1c, a2c, a3c, a4c
        if self.tilt:
            ocp.constraints.idxbu = np.append(ocp.constraints.idxbu, [4, 5, 6, 7])

        # -- Lower Input Bound
        ocp.constraints.lbu = np.array(
            [
                self.params["thrust_min"],
                self.params["thrust_min"],
                self.params["thrust_min"],
                self.params["thrust_min"],
            ]
        )

        if self.tilt:
            ocp.constraints.lbu = np.append(
                ocp.constraints.lbu,
                [
                    self.params["a_min"],
                    self.params["a_min"],
                    self.params["a_min"],
                    self.params["a_min"],
                ],
            )

        # -- Upper Input Bound
        ocp.constraints.ubu = np.array(
            [
                self.params["thrust_max"],
                self.params["thrust_max"],
                self.params["thrust_max"],
                self.params["thrust_max"],
            ]
        )

        if self.tilt:
            ocp.constraints.ubu = np.append(
                ocp.constraints.ubu,
                [
                    self.params["a_max"],
                    self.params["a_max"],
                    self.params["a_max"],
                    self.params["a_max"],
                ],
            )

        # Nonlinear constraint for quaternions
        if self.include_quaternion_constraint:
            # Note: This is theoretically necessary to ensure that the quaternion stays on the unit sphere
            # and represents a valid rotation. In practice, it seems to work quite well without this constraint,
            # and including it may rather reduce solving performance due to higher dimensionality of the optimisation problem!
            ocp.model.con_h_expr = self.qw**2 + self.qx**2 + self.qy**2 + self.qz**2 - 1.0  # ||q||^2 - 1 = 0
            ocp.model.con_h_expr_e = ocp.model.con_h_expr
            ocp.constraints.lh = np.array([0.0])
            ocp.constraints.uh = np.array([0.0])
            ocp.constraints.lh_e = np.array([0.0])
            ocp.constraints.uh_e = np.array([0.0])

        # - Slack variables for soft constraints
        if self.include_soft_constraints:
            # Avoid hard constraint which can lead to improved solver performance
            # Note: Symmetrically for upper and lower bounds
            # -- Indices of slacked constraints within bx
            ocp.constraints.idxsbx = np.arange(len(ocp.constraints.idxbx))
            ocp.constraints.idxsbx_e = np.arange(len(ocp.constraints.idxbx))
            num_constraints = len(ocp.constraints.idxsbx)
            num_constraints_e = len(ocp.constraints.idxsbx_e)
            if self.include_quaternion_constraint:
                ocp.constraints.idxsh = np.array([0])
                ocp.constraints.idxsh_e = np.array([0])
                num_constraints += 1
                num_constraints_e += 1
            # -- Linear term
            ocp.cost.Zl = self.params["linear_slack_weight"] * np.ones((num_constraints,))
            ocp.cost.Zu = self.params["linear_slack_weight"] * np.ones((num_constraints,))
            ocp.cost.Zl_e = self.params["linear_slack_weight"] * np.ones((num_constraints_e,))
            ocp.cost.Zu_e = self.params["linear_slack_weight"] * np.ones((num_constraints_e,))
            # -- Quadratic term
            ocp.cost.zl = self.params["quadratic_slack_weight"] * np.ones((num_constraints,))
            ocp.cost.zu = self.params["quadratic_slack_weight"] * np.ones((num_constraints,))
            ocp.cost.zl_e = self.params["quadratic_slack_weight"] * np.ones((num_constraints_e,))
            ocp.cost.zu_e = self.params["quadratic_slack_weight"] * np.ones((num_constraints_e,))

        # =====================================================================
        # Model parameters
        # Setup N x NP matrix of parameters for each discretization node for temporal networks
        # same order: phy_params = ca.vertcat(mass, gravity, inertia, kq_d_kt, dr, p1_b, p2_b, p3_b, p4_b, t_rotor, t_servo, ee_p, ee_q)
        self.acados_parameters = np.zeros((nn + 1, n_param))
        self.acados_parameters[:, 0:4] = np.array([1, 0, 0, 0])  # For nonlinear quaternion error
        if len(self.phys.physical_param_list) != 31:
            raise ValueError("Physical parameters are not as expected. Please check the physical model.")
        self.acados_parameters[:, 4 : 4 + len(self.phys.physical_param_list)] = np.array(self.phys.physical_param_list)

        ocp.parameter_values = self.acados_parameters[0, :]
        # =====================================================================

        # Reference values
        # Note, these are placeholders and will be updated online during tracking
        x_ref = np.zeros((nx,))
        u_ref = np.zeros((nu,))
        ocp.cost.yref = np.concatenate((x_ref, u_ref))
        ocp.cost.yref_e = x_ref
        ocp.constraints.x0 = x_ref

        # Solver options
        ocp.solver_options.tf = self.params["T_horizon"]
        ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"  # "IPOPT", "FULL_CONDENSING_HPIPM"
        ocp.solver_options.hpipm_mode = "BALANCE"  # "BALANCE", "SPEED_ABS", "SPEED", "ROBUST". Default: "BALANCE".
        # Start up flags:       [Seems only works for FULL_CONDENSING_QPOASES]
        # 0: no warm start; 1: warm start; 2: hot start. Default: 0
        # ocp.solver_options.qp_solver_warm_start = 1
        ocp.solver_options.hessian_approx = "GAUSS_NEWTON"  # "EXACT", "GAUSS_NEWTON"
        ocp.solver_options.integrator_type = "ERK"  # explicit Runge-Kutta integrator
        ocp.solver_options.print_level = 0
        ocp.solver_options.nlp_solver_type = "SQP_RTI"
        ocp.solver_options.qp_solver_cond_N = self.params["N_steps"]

        # Build acados ocp into current working directory (which was created in super class)
        json_file_path = os.path.join("./" + ocp.model.name + "_acados_ocp.json")
        solver = AcadosOcpSolver(ocp, json_file=json_file_path, build=build)
        print("Generated C code for acados solver successfully to " + os.getcwd())

        return solver
    # fmt: on

    def track(self, ocp_solver, xr, ur):
        """
        Tracks a trajectory defined by xr and ur, where xr is the reference state and ur is the reference control input.
        :param xr: Reference state trajectory (N+1, state_dim)
        :param ur: Reference control input trajectory (N, control_dim)
        """
        # 0 ~ N-1
        for j in range(ocp_solver.N):
            yr = np.concatenate((xr[j, :], ur[j, :]))
            ocp_solver.set(j, "yref", yr)
            quaternion_ref = xr[j, 6:10]
            self.acados_parameters[j, 0:4] = quaternion_ref  # For nonlinear quaternion error

        # N
        yr = xr[ocp_solver.N, :]
        ocp_solver.set(ocp_solver.N, "yref", yr)
        quaternion_ref = xr[ocp_solver.N, 6:10]
        self.acados_parameters[ocp_solver.N, 0:4] = quaternion_ref  # For nonlinear quaternion error

    def append_delay(self, delay: int = 0) -> tuple[ca.MX, ca.MX]:
        """
        Append delay of the state and control as parameters to the acados model and solver.
        The additional states and controls are used as input to the neural network to account for temporal dependencies.
        Note, the states and input pairs are defined to be sorted from newest to oldest in parameters.
        :param delay: Number of previous time steps to include as the model input.
        """
        state_prev = ca.MX()
        controls_prev = ca.MX()

        # Store the previous state and control as parameters in the model
        self.delay_start_idx = self.parameters.size()[0]
        for i in range(1, delay):
            # Create symbolic variables for the previous state and control
            state_prev_i = ca.MX.sym(f"state_prev_{i}", self.nx)
            controls_prev_i = ca.MX.sym(f"controls_prev_{i}", self.nu)

            # Concatenate state and input together for each time step
            self.parameters = ca.vertcat(self.parameters, state_prev_i, controls_prev_i)
            state_prev = ca.horzcat(state_prev, state_prev_i)
            controls_prev = ca.horzcat(controls_prev, controls_prev_i)
        self.delay_end_idx = self.parameters.size()[0]
        return state_prev, controls_prev

    def _create_reference_generator(self) -> QDNMPCReferenceGenerator:
        # fmt: off
        # Pass the model's and robot's properties to the reference generator
        return QDNMPCReferenceGenerator(self,
                                        self.phys.p1_b,    self.phys.p2_b, self.phys.p3_b, self.phys.p4_b,
                                        self.phys.dr1,     self.phys.dr2,  self.phys.dr3,  self.phys.dr4,
                                        self.phys.kq_d_kt, self.phys.mass, self.phys.gravity)
        # fmt: on

    def get_reference_generator(self) -> QDNMPCReferenceGenerator:
        return self._reference_generator

    def get_reference(self, target_xyz, target_qwxyz, ft_ref, a_ref) -> tuple[np.ndarray, np.ndarray]:
        """
        Assemble reference trajectory from target pose and reference control values.
        Gets called from reference generator class.
        Note: The definition of the reference is closely linked to the definition of the cost function.
        Therefore, this is explicitly stated in each controller file to increase comprehensiveness.

        :param target_xyz: Target position
        :param target_qwxy: Target quarternions
        :param ft_ref: Target thrust
        :param a_ref: Target servo angles
        :return xr: Reference for the state x
        :return ur: Reference for the input u
        """
        # Get dimensions
        ocp = self.get_ocp()
        nn = ocp.solver_options.N_horizon
        nx = ocp.dims.nx
        nu = ocp.dims.nu

        # Assemble state reference
        xr = np.zeros([nn + 1, nx])
        xr[:, 0] = target_xyz[0]  # x
        xr[:, 1] = target_xyz[1]  # y
        xr[:, 2] = target_xyz[2]  # z
        # No reference for vx, vy, vz (idx: 3, 4, 5)
        xr[:, 6] = target_qwxyz[0]  # qx
        xr[:, 7] = target_qwxyz[1]  # qx
        xr[:, 8] = target_qwxyz[2]  # qy
        xr[:, 9] = target_qwxyz[3]  # qz
        # No reference for wx, wy, wz (idx: 10, 11, 12)
        xr[:, 13] = a_ref[0]
        xr[:, 14] = a_ref[1]
        xr[:, 15] = a_ref[2]
        xr[:, 16] = a_ref[3]

        # Assemble input reference
        # Note: Reference has to be zero if variable is included as state in cost function!
        ur = np.zeros([nn, nu])
        ur[:, 0] = ft_ref[0]
        ur[:, 1] = ft_ref[1]
        ur[:, 2] = ft_ref[2]
        ur[:, 3] = ft_ref[3]

        return xr, ur
