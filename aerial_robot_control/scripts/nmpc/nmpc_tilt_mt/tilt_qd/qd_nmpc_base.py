import os
from abc import abstractmethod
import numpy as np
import casadi as ca
from acados_template import AcadosModel, AcadosOcpSolver, AcadosSim, AcadosSimSolver
from ..rh_base import RecedingHorizonBase
from .qd_reference_generator import QDNMPCReferenceGenerator


class QDNMPCBase(RecedingHorizonBase):
    """
    Base class for all NMPC controllers for quadrotors.
    Inherits from RecedingHorizonBase which also lays foundations for MHE classes.
    """

    def __init__(self, build: bool = True):
        #     The child classes only have specifications which define the controller specifications and need to set the following flags:
        # check if the model name is set
        # - model_name: Name of the model defined in controller file.
        if not hasattr(self, "model_name"):
            raise AttributeError("Model name not set. Please set the model_name attribute in the child class.")
        # - phys: Physical parameters of the robot.
        if not hasattr(self, "phys"):
            raise AttributeError("Physical parameters not set. Please set the phys attribute in the child class.")
        # - num_rotors: Number of rotors for the multirotor system. Default is 4 for quadrotor.
        if not hasattr(self, "num_rotors"):
            self.num_rotors = 4  # Default to quadrotor
        # - tilt: Flag to include tiltable rotors. If not included, the quadrotor is assumed to be a fixed quadrotor.
        if not hasattr(self, "tilt"):
            raise AttributeError("Tilt flag not set. Please set the tilt attribute in the child class.")
        # - include_servo_model: Flag to include the servo model based on the angle alpha (a) between frame E (end of arm) and R (rotor). If not included, angle control is assumed to be equal to angle state.
        if not hasattr(self, "include_servo_model"):
            raise AttributeError(
                "Servo model flag not set. Please set the include_servo_model attribute in the child class."
            )
        # - include_servo_derivative: Flag to include the continuous time-derivative of the servo angle as control input(!) instead of numeric differentation.
        if not hasattr(self, "include_servo_derivative"):
            self.include_servo_derivative = False
        # - include_thrust_model: Flag to include dynamics from rotor and use thrust as state. If not included, thrust control is assumed to be equal to thrust state.
        if not hasattr(self, "include_thrust_model"):
            raise AttributeError(
                "Thrust model flag not set. Please set the include_thrust_model attribute in the child class."
            )

        # Disturbance on each rotor individually was investigated into but didn't properly work, therefore only disturbance on CoG implemented.
        # include_cog_dist_parameter are for I term, which accounts for model error. include_cog_dist_model are for disturbances.
        # - include_cog_dist_parameter: Flag to include disturbance on the CoG into the acados model parameters.
        if not hasattr(self, "include_cog_dist_parameter"):
            raise AttributeError(
                "CoG disturbance parameter flag not set. Please set the include_cog_dist_parameter attribute in the child class."
            )

        # These two variables are only for impedance control
        # - include_cog_dist_model: Flag to include disturbance on the CoG into the acados model state.
        if not hasattr(self, "include_cog_dist_model"):
            self.include_cog_dist_model = False
        # - include_impedance: Flag to include virtual mass and inertia to calculate impedance cost. Doesn't add any functionality for the model.
        if not hasattr(self, "include_impedance"):
            self.include_impedance = False

        self.acados_init_p = None  # initial value for parameters in acados. Mainly for physical parameters.

        # Call RecedingHorizon constructor coming as NMPC method
        super().__init__("nmpc", build)

        # Create Reference Generator object
        self._reference_generator = self._create_reference_generator()

    def get_reference_generator(self) -> QDNMPCReferenceGenerator:
        return self._reference_generator

    def create_acados_model(self) -> AcadosModel:
        """
        Define generic state-space, acados model parameters, control inputs for kinematics of a quadrotor.
        Calculate transformation matrix from robot's architecture to compute internal wrench.
        Assemble acados model based on given cost function.
        """
        # fmt: off
        if self.include_servo_derivative and not self.include_servo_model:
            raise ValueError(
                "Servo derivative can only work with servo angle defined as state through the 'include_servo_model' flag."
            )

        # Standard state-space (Note: store in self to access for cost function in child controller class)
        self.x = ca.SX.sym("x")  # Position
        self.y = ca.SX.sym("y")
        self.z = ca.SX.sym("z")
        self.p = ca.vertcat(self.x, self.y, self.z)
        self.vx = ca.SX.sym("vx")  # Linear velocity
        self.vy = ca.SX.sym("vy")
        self.vz = ca.SX.sym("vz")
        self.v = ca.vertcat(self.vx, self.vy, self.vz)
        self.qw = ca.SX.sym("qw")  # Quaternions
        self.qx = ca.SX.sym("qx")
        self.qy = ca.SX.sym("qy")
        self.qz = ca.SX.sym("qz")
        self.q = ca.vertcat(self.qw, self.qx, self.qy, self.qz)
        self.wx = ca.SX.sym("wx")  # Angular velocity
        self.wy = ca.SX.sym("wy")
        self.wz = ca.SX.sym("wz")
        self.w = ca.vertcat(self.wx, self.wy, self.wz)
        state = ca.vertcat(self.p, self.v, self.q, self.w)

        # - Extend state-space by dynamics of servo angles (actual)
        # Differentiate between actual angles and control angles
        # Note: If servo angle is not used as control input the model for omnidirectional Quadrotor
        # has been observed to be unstable (see https://arxiv.org/abs/2405.09871).
        if self.tilt and self.include_servo_model:
            # Dynamically create servo angle state variables for each rotor
            self.a_s_list = []
            for i in range(1, self.num_rotors + 1):
                a_s = ca.SX.sym(f"a{i}s")
                setattr(self, f"a{i}s", a_s)
                self.a_s_list.append(a_s)
            self.a_s = ca.vertcat(*self.a_s_list)
            state = ca.vertcat(state, self.a_s)

        # - Extend state-space by dynamics of rotor (actual)
        # Differentiate between actual thrust and control thrust
        if self.include_thrust_model:
            # Dynamically create thrust state variables for each rotor
            self.ft_s_list = []
            for i in range(1, self.num_rotors + 1):
                ft_s = ca.SX.sym(f"ft{i}s")
                setattr(self, f"ft{i}s", ft_s)
                self.ft_s_list.append(ft_s)
            self.ft_s = ca.vertcat(*self.ft_s_list)
            state = ca.vertcat(state, self.ft_s)

        # - Extend state-space by disturbance on CoG (actual)
        # Differentiate between actual disturbance set as state and set as parameter
        if self.include_cog_dist_model:
            # Force disturbance applied to CoG in World frame
            self.fds_w = ca.SX.sym("fds_w", 3)
            # Torque disturbance applied to CoG in Body frame
            self.tau_ds_b = ca.SX.sym("tau_ds_b", 3)

            state = ca.vertcat(state, self.fds_w, self.tau_ds_b)
        else:
            self.fds_w = ca.vertcat(0.0, 0.0, 0.0)
            self.tau_ds_b = ca.vertcat(0.0, 0.0, 0.0)

        # Control inputs
        # - Forces from thrust at each rotor
        # Dynamically create thrust control variables for each rotor
        self.ft_c_list = []
        for i in range(1, self.num_rotors + 1):
            ft_c = ca.SX.sym(f"ft{i}c")
            setattr(self, f"ft{i}c", ft_c)
            self.ft_c_list.append(ft_c)
        self.ft_c = ca.vertcat(*self.ft_c_list)
        controls = ca.vertcat(*self.ft_c_list)
        # - Servo angle for tiltable rotors (actuated)
        if self.tilt:
            # Dynamically create servo angle control variables for each rotor
            self.a_c_list = []
            self.ad_c_list = []
            for i in range(1, self.num_rotors + 1):
                a_c = ca.SX.sym(f"a{i}c")
                setattr(self, f"a{i}c", a_c)
                self.a_c_list.append(a_c)
            # Either use the time-derivative of the servo angle as control input directly
            if self.include_servo_derivative:
                self.ad_c = ca.vertcat(*self.a_c_list)
                controls = ca.vertcat(controls, self.ad_c)
            # Or use numerical differentation to calculate time-derivate in dynamical model
            else:
                self.a_c = ca.vertcat(*self.a_c_list)
                controls = ca.vertcat(controls, self.a_c)

        # Model parameters
        self.qwr = ca.SX.sym("qwr")  # Reference for quaternions
        self.qxr = ca.SX.sym("qxr")
        self.qyr = ca.SX.sym("qyr")
        self.qzr = ca.SX.sym("qzr")
        parameters = ca.vertcat(self.qwr, self.qxr, self.qyr, self.qzr)

        # added on 2025-3-28: make physical parameters available in the model
        mass = ca.SX.sym("mass")
        gravity = ca.SX.sym("gravity")

        Ixx = ca.SX.sym("Ixx")
        Iyy = ca.SX.sym("Iyy")
        Izz = ca.SX.sym("Izz")

        kq_d_kt = ca.SX.sym("kq_d_kt")

        # Dynamically create rotor direction and position parameters for each rotor
        dr_list = []
        p_b_list = []
        for i in range(1, self.num_rotors + 1):
            dr = ca.SX.sym(f"dr{i}")
            setattr(self, f"dr{i}", dr)
            dr_list.append(dr)

            p_b = ca.SX.sym(f"p{i}_b", 3)
            setattr(self, f"p{i}_b", p_b)
            p_b_list.append(p_b)

        t_rotor = ca.SX.sym("t_rotor")
        t_servo = ca.SX.sym("t_servo")

        # added on 2025-07-16: add the pose of end-effector
        # position and quaternion of an end-effector in CoG frame
        self.ee_p = ca.SX.sym("ee_p", 3)
        self.ee_q = ca.SX.sym("ee_qwxyz", 4)  # qw, qx, qy, qz

        # Build phy_params by interleaving dr and p_b for each rotor
        phy_params = ca.vertcat(mass, gravity, Ixx, Iyy, Izz, kq_d_kt)
        for i in range(self.num_rotors):
            phy_params = ca.vertcat(phy_params, dr_list[i], p_b_list[i])
        phy_params = ca.vertcat(phy_params, t_rotor, t_servo, self.ee_p, self.ee_q)
        parameters = ca.vertcat(parameters, phy_params)

        # - Extend model parameters by CoG disturbance
        if self.include_cog_dist_parameter:
            # Force disturbance applied to CoG in World frame
            self.fdp_w = ca.SX.sym("fdp_w", 3)
            # Torque disturbance applied to CoG in Body frame
            self.tau_dp_b = ca.SX.sym("tau_dp_b", 3)

            parameters = ca.vertcat(parameters, self.fdp_w, self.tau_dp_b)
        else:
            self.fdp_w = ca.vertcat(0.0, 0.0, 0.0)
            self.tau_dp_b = ca.vertcat(0.0, 0.0, 0.0)

        # - Extend model parameters by virtual mass and inertia for impedance cost function
        if self.include_impedance:
            if not self.include_cog_dist_model or not self.include_cog_dist_parameter:
                raise ValueError("Impedance cost can only be calculated if disturbance flags are activated.")

            mpx = ca.SX.sym("mpx")  # Virtual mass (p = position)
            mpy = ca.SX.sym("mpy")
            mpz = ca.SX.sym("mpz")
            self.mp = ca.vertcat(mpx, mpy, mpz)

            mqx = ca.SX.sym("mqx")  # Virtual inertia (q = quaternion)
            mqy = ca.SX.sym("mqy")
            mqz = ca.SX.sym("mqz")
            self.mq = ca.vertcat(mqx, mqy, mqz)

            parameters = ca.vertcat(parameters, self.mp, self.mq)

        # Transformation matrices between coordinate systems World, Body, End-of-arm, Rotor using quaternions
        # - World to Body
        rot_wb = self._get_rot_wb_ca(self.qw, self.qx, self.qy, self.qz)

        # - Body to End-of-arm (dynamically create for each rotor)
        rot_be_list = []
        for i in range(self.num_rotors):
            p_b = p_b_list[i]
            denominator = ca.sqrt(p_b[0] ** 2 + p_b[1] ** 2)
            rot_be = ca.vertcat(
                ca.horzcat(p_b[0] / denominator, -p_b[1] / denominator, 0),
                ca.horzcat(p_b[1] / denominator,  p_b[0] / denominator, 0),
                ca.horzcat(0, 0, 1)
            )
            rot_be_list.append(rot_be)

        # - End-of-arm to Rotor
        # Take tilt rotation with angle alpha (a) of R frame to E frame into account
        rot_er_list = []
        if self.tilt:
            # If servo dynamics are modeled, use angle state.
            # Else use angle control which is then assumed to be equal to the angle state at all times.
            for i in range(self.num_rotors):
                if self.include_servo_model:
                    a = self.a_s_list[i]
                else:
                    a = self.a_c_list[i]
                rot_er = self._get_rot_around_x(a)
                rot_er_list.append(rot_er)
        else:
            for i in range(self.num_rotors):
                rot_er_list.append(ca.SX.eye(3))

        # Wrench in Rotor frame
        # If rotor dynamics are modeled, explicitly use thrust state as force.
        # Else use thrust control which is then assumed to be equal to the thrust state at all times.
        ft_list = []
        for i in range(self.num_rotors):
            if self.include_thrust_model:
                ft = self.ft_s_list[i]
            else:
                ft = self.ft_c_list[i]
            ft_list.append(ft)

        # Force and torque vectors in rotor frame for each rotor
        ft_r_list = []
        tau_r_list = []
        for i in range(self.num_rotors):
            ft_r = ca.vertcat(0, 0, ft_list[i])
            tau_r = ca.vertcat(0, 0, -dr_list[i] * ft_list[i] * kq_d_kt)
            ft_r_list.append(ft_r)
            tau_r_list.append(tau_r)

        # Wrench in Body frame - sum contributions from all rotors
        fu_b = ca.SX.zeros(3)
        tau_u_b = ca.SX.zeros(3)
        for i in range(self.num_rotors):
            # Force contribution
            fu_b += ca.mtimes(rot_be_list[i], ca.mtimes(rot_er_list[i], ft_r_list[i]))
            # Torque contribution (from rotor drag + moment arm)
            tau_u_b += ca.mtimes(rot_be_list[i], ca.mtimes(rot_er_list[i], tau_r_list[i]))
            tau_u_b += ca.cross(p_b_list[i], ca.mtimes(rot_be_list[i], ca.mtimes(rot_er_list[i], ft_r_list[i])))


        # Compute Inertia
        I = ca.diag(ca.vertcat(Ixx, Iyy, Izz))
        I_inv = ca.diag(ca.vertcat(1 / Ixx, 1 / Iyy, 1 / Izz))
        g_w = ca.vertcat(0, 0, -gravity)  # World frame

        # Dynamic model (Time-derivative of state)
        ds = ca.vertcat(
            self.v,
            (ca.mtimes(rot_wb, fu_b) + self.fds_w + self.fdp_w) / mass + g_w,
            (-self.wx * self.qx - self.wy * self.qy - self.wz * self.qz) / 2,
            ( self.wx * self.qw + self.wz * self.qy - self.wy * self.qz) / 2,
            ( self.wy * self.qw - self.wz * self.qx + self.wx * self.qz) / 2,
            ( self.wz * self.qw + self.wy * self.qx - self.wx * self.qy) / 2,
            ca.mtimes(I_inv, (-ca.cross(self.w, ca.mtimes(I, self.w)) + tau_u_b + self.tau_ds_b + self.tau_dp_b)),
        )

        # - Extend model by servo first-order dynamics
        # Assumption if not included: a_c = a_s
        # Either use continuous time-derivate as control variable
        if self.include_servo_derivative:
            ds = ca.vertcat(ds,
                            self.ad_c
                            )
        # Or use numerical differentation
        if self.include_servo_model and not self.include_servo_derivative:
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
                            ca.vertcat(0.0, 0.0, 0.0),
                            ca.vertcat(0.0, 0.0, 0.0),
                            )

        # Assemble acados function
        f = ca.Function("f", [state, controls], [ds], ["state", "control_input"], ["ds"], {"allow_free": True})

        # Implicit dynamics
        # Note: Used only mainly because of acados template
        x_dot = ca.SX.sym("x_dot", state.size())  # Combined state vector
        f_impl = x_dot - f(state, controls)

        # Get terms of cost function
        if self.include_impedance:
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
        model.cost_y_expr = ca.vertcat(state_y, control_y)  # NONLINEAR_LS
        model.cost_y_expr_e = state_y_e

        return model
        # fmt: on

    @staticmethod
    def _get_rot_around_x(angle):  # TODO: change these parts to some library
        """
        Returns the rotation matrix for a rotation around the X axis by the given angle.
        """
        return ca.vertcat(
            ca.horzcat(1, 0, 0),
            ca.horzcat(0, ca.cos(angle), -ca.sin(angle)),
            ca.horzcat(0, ca.sin(angle), ca.cos(angle)),
        )

    @staticmethod
    def _get_rot_wb_ca(qw, qx, qy, qz):
        row_1 = ca.horzcat(
            ca.SX(1 - 2 * qy**2 - 2 * qz**2), ca.SX(2 * qx * qy - 2 * qw * qz), ca.SX(2 * qx * qz + 2 * qw * qy)
        )
        row_2 = ca.horzcat(
            ca.SX(2 * qx * qy + 2 * qw * qz), ca.SX(1 - 2 * qx**2 - 2 * qz**2), ca.SX(2 * qy * qz - 2 * qw * qx)
        )
        row_3 = ca.horzcat(
            ca.SX(2 * qx * qz - 2 * qw * qy), ca.SX(2 * qy * qz + 2 * qw * qx), ca.SX(1 - 2 * qx**2 - 2 * qy**2)
        )
        rot_wb = ca.vertcat(row_1, row_2, row_3)
        return rot_wb

    @staticmethod
    def _get_rot_wb_np(qw, qx, qy, qz):
        """
        Create a rotation matrix from quaternions.
        :param qw: Quaternion component w.
        :param qx: Quaternion component x.
        :param qy: Quaternion component y.
        :param qz: Quaternion component z.
        :return: Rotation matrix from World to Body frame.
        """
        return np.array(
            [
                [1 - 2 * (qy**2 + qz**2), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
                [2 * (qx * qy + qw * qz), 1 - 2 * (qx**2 + qz**2), 2 * (qy * qz - qw * qx)],
                [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx**2 + qy**2)],
            ]
        )

    @staticmethod
    def _get_skew_symmetric_matrix(vector):
        """
        Create a skew-symmetric matrix from a 3D vector.
        :param vector: 3D vector to create the skew-symmetric matrix from.
        :return: Skew-symmetric matrix.
        """
        skew_mtx = ca.vertcat(
            ca.horzcat(0, -vector[2], vector[1]),
            ca.horzcat(vector[2], 0, -vector[0]),
            ca.horzcat(-vector[1], vector[0], 0),
        )

        return skew_mtx

    @staticmethod
    def _quaternion_multiply(qw1, qx1, qy1, qz1, qw2, qx2, qy2, qz2):
        """
        Multiply two quaternions.
        """
        return (
            qw1 * qw2 - qx1 * qx2 - qy1 * qy2 - qz1 * qz2,
            qw1 * qx2 + qx1 * qw2 + qy1 * qz2 - qz1 * qy2,
            qw1 * qy2 - qx1 * qz2 + qy1 * qw2 + qz1 * qx2,
            qw1 * qz2 + qx1 * qy2 - qy1 * qx2 + qz1 * qw2,
        )

    @abstractmethod
    def get_weights(self):
        pass

    @abstractmethod
    def get_cost_function(self, lin_acc_w=None, ang_acc_b=None):
        pass

    def create_acados_ocp_solver(self, build: bool = True) -> AcadosOcpSolver:
        """
        Create generic acados solver for NMPC framework of a quadrotor.
        Generate c code into source folder in aerial_robot_control to be used in workflow.
        """
        # Get OCP object
        ocp = super().get_ocp()

        # Model dimensions
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
        # TODO include fixed rotor arch
        # - State box constraints bx
        # -- Index for vx, vy, vz, wx, wy, wz
        ocp.constraints.idxbx = np.array([3, 4, 5, 10, 11, 12])

        # Calculate starting indices for servo and thrust states
        base_state_size = 13  # p(3) + v(3) + q(4) + w(3)
        servo_start_idx = base_state_size
        thrust_start_idx = base_state_size + (self.num_rotors if (self.tilt and self.include_servo_model) else 0)

        # -- Index for a1s, a2s, ..., aNs
        if self.tilt and self.include_servo_model:
            servo_indices = np.arange(servo_start_idx, servo_start_idx + self.num_rotors)
            ocp.constraints.idxbx = np.append(ocp.constraints.idxbx, servo_indices)

            # -- Index for ft1s, ft2s, ..., ftNs (When included servo AND thrust, add further indices)
            if self.include_thrust_model:
                thrust_indices = np.arange(thrust_start_idx, thrust_start_idx + self.num_rotors)
                ocp.constraints.idxbx = np.append(ocp.constraints.idxbx, thrust_indices)

        # -- Index for ft1s, ft2s, ..., ftNs (When only included thrust, use the same indices)
        elif self.include_thrust_model:
            thrust_indices = np.arange(thrust_start_idx, thrust_start_idx + self.num_rotors)
            ocp.constraints.idxbx = np.append(ocp.constraints.idxbx, thrust_indices)

        # -- Lower State Bound
        # fmt: off
        ocp.constraints.lbx = np.array(
            [self.params["v_min"],
             self.params["v_min"],
             self.params["v_min"],
             self.params["w_min"],
             self.params["w_min"],
             self.params["w_min"]])

        if self.tilt and self.include_servo_model:
            ocp.constraints.lbx = np.append(ocp.constraints.lbx,
                                            [self.params["a_min"]] * self.num_rotors)

        if self.include_thrust_model:
            ocp.constraints.lbx = np.append(ocp.constraints.lbx,
                                            [self.params["thrust_min"]] * self.num_rotors)

        # -- Upper State Bound
        ocp.constraints.ubx = np.array(
            [self.params["v_max"],
             self.params["v_max"],
             self.params["v_max"],
             self.params["w_max"],
             self.params["w_max"],
             self.params["w_max"]])

        if self.tilt and self.include_servo_model:
            ocp.constraints.ubx = np.append(ocp.constraints.ubx,
                                            [self.params["a_max"]] * self.num_rotors)

        if self.include_thrust_model:
            ocp.constraints.ubx = np.append(ocp.constraints.ubx,
                                            [self.params["thrust_max"]] * self.num_rotors)

        # - Terminal state box constraints bx_e
        # -- Index for vx, vy, vz, wx, wy, wz
        ocp.constraints.idxbx_e = np.array([3, 4, 5, 10, 11, 12])

        # -- Index for a1s, a2s, ..., aNs
        if self.tilt and self.include_servo_model:
            ocp.constraints.idxbx_e = np.append(ocp.constraints.idxbx_e, servo_indices)

            # -- Index for ft1s, ft2s, ..., ftNs (When included servo AND thrust, add further indices)
            if self.include_thrust_model:
                ocp.constraints.idxbx_e = np.append(ocp.constraints.idxbx_e, thrust_indices)

        # -- Index for ft1s, ft2s, ..., ftNs (When only included thrust, use the same indices)
        elif self.include_thrust_model:
            ocp.constraints.idxbx_e = np.append(ocp.constraints.idxbx_e, thrust_indices)

        # -- Lower Terminal State Bound
        ocp.constraints.lbx_e = np.array(
            [self.params["v_min"],
             self.params["v_min"],
             self.params["v_min"],
             self.params["w_min"],
             self.params["w_min"],
             self.params["w_min"]])

        if self.tilt and self.include_servo_model:
            ocp.constraints.lbx_e = np.append(ocp.constraints.lbx_e,
                [self.params["a_min"]] * self.num_rotors)

        if self.include_thrust_model:
            ocp.constraints.lbx_e = np.append(ocp.constraints.lbx_e,
                [self.params["thrust_min"]] * self.num_rotors)

        # -- Upper Terminal State Bound
        ocp.constraints.ubx_e = np.array(
            [self.params["v_max"],
             self.params["v_max"],
             self.params["v_max"],
             self.params["w_max"],
             self.params["w_max"],
             self.params["w_max"]])

        if self.tilt and self.include_servo_model:
            ocp.constraints.ubx_e = np.append(ocp.constraints.ubx_e,
                [self.params["a_max"]] * self.num_rotors)

        if self.include_thrust_model:
            ocp.constraints.ubx_e = np.append(ocp.constraints.ubx_e,
                [self.params["thrust_max"]] * self.num_rotors)

        # - Input box constraints bu
        # TODO Potentially a good idea to omit the input constraint when set the equivalent state
        # -- Index for ft1c, ft2c, ..., ftNc
        ocp.constraints.idxbu = np.arange(0, self.num_rotors)
        # -- Index for a1c, a2c, ..., aNc
        if self.tilt:
            ocp.constraints.idxbu = np.append(ocp.constraints.idxbu, np.arange(self.num_rotors, 2 * self.num_rotors))

        # -- Lower Input Bound
        ocp.constraints.lbu = np.array([self.params["thrust_min"]] * self.num_rotors)

        if self.tilt:
            ocp.constraints.lbu = np.append(ocp.constraints.lbu,
                [self.params["a_min"]] * self.num_rotors)

        # -- Upper Input Bound
        ocp.constraints.ubu = np.array([self.params["thrust_max"]] * self.num_rotors)

        if self.tilt:
            ocp.constraints.ubu = np.append(ocp.constraints.ubu,
                [self.params["a_max"]] * self.num_rotors)
        # fmt: on

        # Initial state and reference: Set all values such that robot is hovering
        x_ref = np.zeros(nx)
        x_ref[6] = 1.0  # Quaternion qw

        # Set thrust reference for hovering (distributed equally among rotors)
        thrust_hover = self.phys.mass * self.phys.gravity / self.num_rotors
        if self.tilt:
            # When included servo AND thrust, use further indices
            if self.include_servo_model and self.include_thrust_model:
                x_ref[thrust_start_idx : thrust_start_idx + self.num_rotors] = thrust_hover
            # When only included thrust, use the same indices
            elif self.include_thrust_model:
                x_ref[thrust_start_idx : thrust_start_idx + self.num_rotors] = thrust_hover
        else:
            x_ref[thrust_start_idx : thrust_start_idx + self.num_rotors] = thrust_hover

        ocp.constraints.x0 = x_ref  # TODO this should be set in control loop and updated before each solver call

        # Note: This is not really necessary, since the reference is always updated before solver is called
        u_ref = np.zeros(nu)
        # Obeserved to be worse than zero!
        u_ref[0 : self.num_rotors] = thrust_hover  # ft1c, ft2c, ..., ftNc
        ocp.cost.yref = np.concatenate((x_ref, u_ref))
        ocp.cost.yref_e = x_ref

        # Model parameters
        # same order: phy_params = ca.vertcat(mass, gravity, inertia, kq_d_kt, dr, p1_b, p2_b, p3_b, p4_b, t_rotor, t_servo)
        self.acados_init_p = np.zeros(n_param)
        self.acados_init_p[0] = x_ref[6]  # qw
        expected_param_length = 6 + 4 * self.num_rotors + 2 + 7
        # mass, gravity, inertia(3), kq_d_kt + 4*num_rotors*(dr+p_b(3)) + t_rotor, t_servo + ee_p(3), ee_q(4)
        if len(self.phys.physical_param_list) != expected_param_length:
            raise ValueError(
                f"Physical parameters length mismatch. Expected {expected_param_length} for {self.num_rotors} rotors, got {len(self.phys.physical_param_list)}."
            )
        self.acados_init_p[4 : 4 + len(self.phys.physical_param_list)] = np.array(self.phys.physical_param_list)

        ocp.parameter_values = self.acados_init_p

        # Solver options
        ocp.solver_options.tf = self.params["T_horizon"]
        ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"  # "IPOPT", "FULL_CONDENSING_HPIPM"
        ocp.solver_options.hpipm_mode = "BALANCE"  # "BALANCE", "SPEED_ABS", "SPEED", "ROBUST". Default: "BALANCE".
        # Start up flags:       [Seems only works for FULL_CONDENSING_QPOASES]
        # 0: no warm start; 1: warm start; 2: hot start. Default: 0
        # ocp.solver_options.qp_solver_warm_start = 1
        ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
        ocp.solver_options.integrator_type = "ERK"  # explicit Runge-Kutta integrator
        ocp.solver_options.print_level = 0
        ocp.solver_options.nlp_solver_type = "SQP_RTI"
        ocp.solver_options.qp_solver_cond_N = self.params["N_steps"]

        # Build acados ocp into current working directory (which was created in super class)
        json_file_path = os.path.join("./" + ocp.model.name + "_acados_ocp.json")
        solver = AcadosOcpSolver(ocp, json_file=json_file_path, build=build)
        print("Generated C code for acados solver successfully to " + os.getcwd())

        return solver

    @abstractmethod
    def get_reference(self):
        pass

    def _create_reference_generator(self) -> QDNMPCReferenceGenerator:
        # fmt: off
        # Pass the model's and robot's properties to the reference generator
        # Dynamically gather rotor position and direction parameters
        p_b_list = [getattr(self.phys, f"p{i}_b") for i in range(1, self.num_rotors + 1)]
        dr_list = [getattr(self.phys, f"dr{i}") for i in range(1, self.num_rotors + 1)]

        return QDNMPCReferenceGenerator(self,
                                        *p_b_list,  # p1_b, p2_b, ..., pN_b
                                        *dr_list,   # dr1, dr2, ..., drN
                                        self.phys.kq_d_kt, self.phys.mass, self.phys.gravity)
        # fmt: on

    def create_acados_sim_solver(self, ts_sim: float, build: bool = True) -> AcadosSimSolver:
        ocp_model = super().get_acados_model()

        acados_sim = AcadosSim()
        acados_sim.model = ocp_model

        n_param = ocp_model.p.size()[0]
        # same order: phy_params = ca.vertcat(mass, gravity, inertia, kq_d_kt, dr, p1_b, p2_b, p3_b, p4_b, t_rotor, t_servo)
        self.acados_init_p = np.zeros(n_param)
        self.acados_init_p[0] = 1.0  # qw
        self.acados_init_p[4 : 4 + len(self.phys.physical_param_list)] = np.array(self.phys.physical_param_list)
        acados_sim.parameter_values = self.acados_init_p

        acados_sim.solver_options.T = ts_sim
        return AcadosSimSolver(acados_sim, json_file=ocp_model.name + "_acados_sim.json", build=build)
