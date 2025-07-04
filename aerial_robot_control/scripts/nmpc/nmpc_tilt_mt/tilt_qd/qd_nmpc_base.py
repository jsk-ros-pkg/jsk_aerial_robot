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
        # - tilt: Flag to include tiltable rotors. If not included, the quadrotor is assumed to be a fixed quadrotor.
        if not hasattr(self, "tilt"):
            raise AttributeError("Tilt flag not set. Please set the tilt attribute in the child class.")
        # - include_servo_model: Flag to include the servo model based on the angle alpha (a) between frame E (end of arm) and R (rotor). If not included, angle control is assumed to be equal to angle state.
        if not hasattr(self, "include_servo_model"):
            raise AttributeError(
                "Servo model flag not set. Please set the include_servo_model attribute in the child class.")
        # - include_servo_derivative: Flag to include the continuous time-derivative of the servo angle as control input(!) instead of numeric differentation.
        if not hasattr(self, "include_servo_derivative"):
            self.include_servo_derivative = False
        # - include_thrust_model: Flag to include dynamics from rotor and use thrust as state. If not included, thrust control is assumed to be equal to thrust state.
        if not hasattr(self, "include_thrust_model"):
            raise AttributeError(
                "Thrust model flag not set. Please set the include_thrust_model attribute in the child class.")

        # Disturbance on each rotor individually was investigated into but didn't properly work, therefore only disturbance on CoG implemented.
        # include_cog_dist_parameter are for I term, which accounts for model error. include_cog_dist_model are for disturbances.
        # - include_cog_dist_parameter: Flag to include disturbance on the CoG into the acados model parameters.
        if not hasattr(self, "include_cog_dist_parameter"):
            raise AttributeError(
                "CoG disturbance parameter flag not set. Please set the include_cog_dist_parameter attribute in the child class.")

        # These two variables are only for impedance control
        # - include_cog_dist_model: Flag to include disturbance on the CoG into the acados model state.
        if not hasattr(self, "include_cog_dist_model"):
            self.include_cog_dist_model = False
        # - include_impedance: Flag to include virtual mass and inertia to calculate impedance cost. Doesn't add any functionality for the model.
        if not hasattr(self, "include_impedance"):
            self.include_impedance = False

        # - include_quaternion_constraint: Flag to include unit quaternion constraint.
        if not hasattr(self, "include_quaternion_constraint"):
            self.include_quaternion_constraint = False

        # - include_soft_constraints: Flag to include soft constraints for the state.
        if not hasattr(self, "include_soft_constraints"):
            self.include_soft_constraints = False

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
        if self.include_servo_derivative and not self.include_servo_model: raise ValueError(
            "Servo derivative can only work with servo angle defined as state through the 'include_servo_model' flag.")

        # Standard state-space (Note: store in self to access for cost function in child controller class)
        self.p = ca.MX.sym("p", 3)  # Position
        self.v = ca.MX.sym("v", 3)  # Linear velocity
        self.q = ca.MX.sym("q", 4)  # Quaternion
        self.qw = self.q[0]
        self.qx = self.q[1]
        self.qy = self.q[2]
        self.qz = self.q[3]
        self.w = ca.MX.sym("w", 3)  # Angular velocity
        self.wx = self.w[0]
        self.wy = self.w[1]
        self.wz = self.w[2]
        state = ca.vertcat(self.p, self.v, self.q, self.w)

        # - Extend state-space by dynamics of servo angles (actual)
        # Differentiate between actual angles and control angles
        # Note: If servo angle is not used as control input the model for omnidirectional Quadrotor
        # has been observed to be unstable (see https://arxiv.org/abs/2405.09871).
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
            # Either use the time-derivative of the servo angle as control input directly
            if self.include_servo_derivative:
                self.ad_c = ca.MX.sym("ad_c", 4)
                controls = ca.vertcat(controls, self.ad_c)
            # Or use numerical differentiation to calculate time-derivate in dynamical model
            else:
                self.a_c = ca.MX.sym("a_c", 4)
                controls = ca.vertcat(controls, self.a_c)

        # Model parameters
        self.qwr = ca.MX.sym("qwr")  # Reference for quaternions
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

        # dr = ca.MX.sym("dr", 4)         # Distance from CoG to rotor in Body frame [dr1, dr2, dr3, dr4]

        # # TODO dont actually use z-dimensions! -> omit
        # p1_b = ca.MX.sym("p1_b", 3)
        # p2_b = ca.MX.sym("p2_b", 3)
        # p3_b = ca.MX.sym("p3_b", 3)
        # p4_b = ca.MX.sym("p4_b", 3)
        # p_b = ca.horzcat(p1_b, p2_b, p3_b, p4_b).T    # Positions of rotors in Body frame

        # t_rotor = ca.MX.sym("t_rotor")
        # t_servo = ca.MX.sym("t_servo")

        # phy_params = ca.vertcat(mass, gravity, Ixx, Iyy, Izz, kq_d_kt,
        #                         dr, p1_b, p2_b, p3_b, p4_b, t_rotor, t_servo)

        dr1 = ca.MX.sym("dr1")
        dr2 = ca.MX.sym("dr2")
        dr3 = ca.MX.sym("dr3")
        dr4 = ca.MX.sym("dr4")
        dr = ca.vertcat(dr1, dr2, dr3, dr4)

        p1_b = ca.MX.sym("p1_b", 3)
        p2_b = ca.MX.sym("p2_b", 3)
        p3_b = ca.MX.sym("p3_b", 3)
        p4_b = ca.MX.sym("p4_b", 3)
        p_b = ca.horzcat(p1_b, p2_b, p3_b, p4_b).T

        t_rotor = ca.MX.sym("t_rotor")
        t_servo = ca.MX.sym("t_servo")

        phy_params = ca.vertcat(mass, gravity, Ixx, Iyy, Izz, kq_d_kt,
                                dr1, p1_b, dr2, p2_b, dr3, p3_b, dr4, p4_b, t_rotor, t_servo)

        parameters = ca.vertcat(parameters, phy_params)

        # - Extend model parameters by CoG disturbance
        if self.include_cog_dist_parameter:
            # Force disturbance applied to CoG in World frame
            self.fdp_w = ca.MX.sym("fdp_w", 3)
            # Torque disturbance applied to CoG in Body frame
            self.tau_dp_b = ca.MX.sym("tau_dp_b", 3)

            parameters = ca.vertcat(parameters, self.fdp_w, self.tau_dp_b)
        else:
            self.fdp_w = ca.MX.zeros(3)
            self.tau_dp_b = ca.MX.zeros(3)

        # - Extend model parameters by virtual mass and inertia for impedance cost function
        if self.include_impedance:
            if not self.include_cog_dist_model or not self.include_cog_dist_parameter: raise ValueError(
                "Impedance cost can only be calculated if disturbance flags are activated.")
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

        if True:
            # Transformation matrices between coordinate systems World, Body, End-of-arm, Rotor using quaternions
            # - World to Body
            row_1 = ca.horzcat(
                ca.MX(1 - 2 * self.qy ** 2 - 2 * self.qz ** 2), ca.MX(2 * self.qx * self.qy - 2 * self.qw * self.qz),
                ca.MX(2 * self.qx * self.qz + 2 * self.qw * self.qy)
            )
            row_2 = ca.horzcat(
                ca.MX(2 * self.qx * self.qy + 2 * self.qw * self.qz), ca.MX(1 - 2 * self.qx ** 2 - 2 * self.qz ** 2),
                ca.MX(2 * self.qy * self.qz - 2 * self.qw * self.qx)
            )
            row_3 = ca.horzcat(
                ca.MX(2 * self.qx * self.qz - 2 * self.qw * self.qy), ca.MX(2 * self.qy * self.qz + 2 * self.qw * self.qx),
                ca.MX(1 - 2 * self.qx ** 2 - 2 * self.qy ** 2)
            )
            rot_wb = ca.vertcat(row_1, row_2, row_3)
            # - Body to End-of-arm
            denominator = np.sqrt(p1_b[0] ** 2 + p1_b[1] ** 2)
            rot_be1 = ca.vertcat(
                ca.horzcat(p1_b[0] / denominator, -p1_b[1] / denominator, 0),
                ca.horzcat(p1_b[1] / denominator,  p1_b[0] / denominator, 0),
                ca.horzcat(0, 0, 1)
            )

            denominator = np.sqrt(p2_b[0] ** 2 + p2_b[1] ** 2)
            rot_be2 = ca.vertcat(
                ca.horzcat(p2_b[0] / denominator, -p2_b[1] / denominator, 0),
                ca.horzcat(p2_b[1] / denominator,  p2_b[0] / denominator, 0),
                ca.horzcat(0, 0, 1)
            )

            denominator = np.sqrt(p3_b[0] ** 2 + p3_b[1] ** 2)
            rot_be3 = ca.vertcat(
                ca.horzcat(p3_b[0] / denominator, -p3_b[1] / denominator, 0),
                ca.horzcat(p3_b[1] / denominator,  p3_b[0] / denominator, 0),
                ca.horzcat(0, 0, 1)
            )

            denominator = np.sqrt(p4_b[0] ** 2 + p4_b[1] ** 2)
            rot_be4 = ca.vertcat(
                ca.horzcat(p4_b[0] / denominator, -p4_b[1] / denominator, 0),
                ca.horzcat(p4_b[1] / denominator,  p4_b[0] / denominator, 0),
                ca.horzcat(0, 0, 1)
            )

            # - End-of-arm to Rotor
            # Take tilt rotation with angle alpha (a) of R frame to E frame into account
            if self.tilt:
                # If servo dynamics are modeled, use angle state.
                # Else use angle control which is then assumed to be equal to the angle state at all times.
                if self.include_servo_model:
                    a1 = self.a_s[0];
                    a2 = self.a_s[1];
                    a3 = self.a_s[2];
                    a4 = self.a_s[3];
                else:
                    a1 = self.a1c;
                    a2 = self.a2c;
                    a3 = self.a3c;
                    a4 = self.a4c

                rot_e1r1 = ca.vertcat(
                    ca.horzcat(1, 0, 0),
                    ca.horzcat(0, ca.cos(a1), -ca.sin(a1)),
                    ca.horzcat(0, ca.sin(a1),  ca.cos(a1))
                )
                rot_e2r2 = ca.vertcat(
                    ca.horzcat(1, 0, 0), ca.horzcat(0, ca.cos(a2), -ca.sin(a2)), ca.horzcat(0, ca.sin(a2), ca.cos(a2))
                )
                rot_e3r3 = ca.vertcat(
                    ca.horzcat(1, 0, 0), ca.horzcat(0, ca.cos(a3), -ca.sin(a3)), ca.horzcat(0, ca.sin(a3), ca.cos(a3))
                )
                rot_e4r4 = ca.vertcat(
                    ca.horzcat(1, 0, 0), ca.horzcat(0, ca.cos(a4), -ca.sin(a4)), ca.horzcat(0, ca.sin(a4), ca.cos(a4))
                )
            else:
                rot_e1r1 = ca.MX.eye(3);
                rot_e2r2 = ca.MX.eye(3);
                rot_e3r3 = ca.MX.eye(3);
                rot_e4r4 = ca.MX.eye(3)

            # Wrench in Rotor frame
            # If rotor dynamics are modeled, explicitly use thrust state as force.
            # Else use thrust control which is then assumed to be equal to the thrust state at all times.
            if self.include_thrust_model:
                ft1 = self.ft_s[0];
                ft2 = self.ft_s[1];
                ft3 = self.ft_s[2];
                ft4 = self.ft_s[3];
            else:
                ft1 = self.ft_c[0];
                ft2 = self.ft_c[1];
                ft3 = self.ft_c[2];
                ft4 = self.ft_c[3];

            ft_r1 = ca.vertcat(0, 0, ft1)
            ft_r2 = ca.vertcat(0, 0, ft2)
            ft_r3 = ca.vertcat(0, 0, ft3)
            ft_r4 = ca.vertcat(0, 0, ft4)

            tau_r1 = ca.vertcat(0, 0, -dr1 * ft1 * kq_d_kt)
            tau_r2 = ca.vertcat(0, 0, -dr2 * ft2 * kq_d_kt)
            tau_r3 = ca.vertcat(0, 0, -dr3 * ft3 * kq_d_kt)
            tau_r4 = ca.vertcat(0, 0, -dr4 * ft4 * kq_d_kt)

            # Wrench in Body frame
            fu_b = (
                    ca.mtimes(rot_be1, ca.mtimes(rot_e1r1, ft_r1))
                    + ca.mtimes(rot_be2, ca.mtimes(rot_e2r2, ft_r2))
                    + ca.mtimes(rot_be3, ca.mtimes(rot_e3r3, ft_r3))
                    + ca.mtimes(rot_be4, ca.mtimes(rot_e4r4, ft_r4))
            )
            fu_w = rot_wb @ fu_b
            tau_u_b = (
                    ca.mtimes(rot_be1, ca.mtimes(rot_e1r1, tau_r1))
                    + ca.mtimes(rot_be2, ca.mtimes(rot_e2r2, tau_r2))
                    + ca.mtimes(rot_be3, ca.mtimes(rot_e3r3, tau_r3))
                    + ca.mtimes(rot_be4, ca.mtimes(rot_e4r4, tau_r4))
                    + ca.cross(p1_b, ca.mtimes(rot_be1, ca.mtimes(rot_e1r1, ft_r1)))
                    + ca.cross(p2_b, ca.mtimes(rot_be2, ca.mtimes(rot_e2r2, ft_r2)))
                    + ca.cross(p3_b, ca.mtimes(rot_be3, ca.mtimes(rot_e3r3, ft_r3)))
                    + ca.cross(p4_b, ca.mtimes(rot_be4, ca.mtimes(rot_e4r4, ft_r4)))
            )

        else:

            # ===================================================================================================
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
            # These represent the torques due to thrust forces acting at distance from CoG
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
            # ===================================================================================================

        # Compute Inertia
        I = ca.diag(ca.vertcat(Ixx, Iyy, Izz))
        I_inv = ca.diag(ca.vertcat(1 / Ixx, 1 / Iyy, 1 / Izz))
        g_w = ca.vertcat(0, 0, -gravity)  # World frame

        # Dynamic model (Time-derivative of state)
        ds = ca.vertcat(
            self.v,
            (fu_w + self.fds_w + self.fdp_w) / mass + g_w,
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
        x_dot = ca.MX.sym("x_dot", state.size())  # Combined state vector
        f_impl = x_dot - f(state, controls)

        # Get terms of cost function
        if self.include_impedance:
            # Compute linear acceleration (in World frame) and angular acceleration (in Body frame) for impedance cost
            # Note: the wrench from I Term and Wrench Est are all important for this term. If we don't consider I Term,
            # a constant disturbance will be injected.
            lin_acc_w = (fu_w + self.fds_w + self.fdp_w) / mass + g_w
            ang_acc_b = ca.mtimes(I_inv,
                                  (-ca.cross(self.w, ca.mtimes(I, self.w)) + tau_u_b + self.tau_ds_b + self.tau_dp_b))

            state_y, state_y_e, control_y = self.get_cost_function(lin_acc_w=lin_acc_w, ang_acc_b=ang_acc_b)
        else:
            state_y, state_y_e, control_y = self.get_cost_function()

        # Assemble acados model
        model = AcadosModel()
        model.name = self.model_name
        model.f_expl_expr = f(state, controls)  # CasADi expression for the explicit dynamics
        model.f_impl_expr = f_impl               # CasADi expression for the implicit dynamics
        model.x = state
        model.xdot = x_dot
        model.u = controls
        model.p = parameters
        model.cost_y_expr = ca.vertcat(state_y, control_y)  # NONLINEAR_LS
        model.cost_y_expr_e = state_y_e

        return model

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

        # -- Index for a1s, a2s, a3s, a4s
        if self.tilt and self.include_servo_model:
            ocp.constraints.idxbx = np.append(ocp.constraints.idxbx, [13, 14, 15, 16])

            # -- Index for ft1s, ft2s, ft3s, ft4s (When included servo AND thrust, add further indices)
            if self.include_thrust_model:
                ocp.constraints.idxbx = np.append(ocp.constraints.idxbx, [17, 18, 19, 20])

        # -- Index for ft1s, ft2s, ft3s, ft4s (When only included thrust, use the same indices)
        elif self.include_thrust_model:
            ocp.constraints.idxbx = np.append(ocp.constraints.idxbx, [13, 14, 15, 16])

        # -- Lower State Bound
        ocp.constraints.lbx = np.array(
            [self.params["v_min"],
             self.params["v_min"],
             self.params["v_min"],
             self.params["w_min"],
             self.params["w_min"],
             self.params["w_min"]])

        if self.tilt and self.include_servo_model:
            ocp.constraints.lbx = np.append(ocp.constraints.lbx,
                                            [self.params["a_min"],
                                             self.params["a_min"],
                                             self.params["a_min"],
                                             self.params["a_min"]])

        if self.include_thrust_model:
            ocp.constraints.lbx = np.append(ocp.constraints.lbx,
                                            [self.params["thrust_min"],
                                             self.params["thrust_min"],
                                             self.params["thrust_min"],
                                             self.params["thrust_min"]])

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
                                            [self.params["a_max"],
                                             self.params["a_max"],
                                             self.params["a_max"],
                                             self.params["a_max"]])

        if self.include_thrust_model:
            ocp.constraints.ubx = np.append(ocp.constraints.ubx,
                                            [self.params["thrust_max"],
                                             self.params["thrust_max"],
                                             self.params["thrust_max"],
                                             self.params["thrust_max"]])

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
                [self.params["a_min"],
                 self.params["a_min"],
                 self.params["a_min"],
                 self.params["a_min"]])

        if self.include_thrust_model:
            ocp.constraints.lbx_e = np.append(ocp.constraints.lbx_e,
                [self.params["thrust_min"],
                 self.params["thrust_min"],
                 self.params["thrust_min"],
                 self.params["thrust_min"]])

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
                [self.params["a_max"],
                 self.params["a_max"],
                 self.params["a_max"],
                 self.params["a_max"]])

        if self.include_thrust_model:
            ocp.constraints.ubx_e = np.append(ocp.constraints.ubx_e,
                [self.params["thrust_max"],
                 self.params["thrust_max"],
                 self.params["thrust_max"],
                 self.params["thrust_max"]])

        # - Input box constraints bu
        # TODO Potentially a good idea to omit the input constraint when set the equivalent state
        # -- Index for ft1c, ft2c, ft3c, ft4c
        ocp.constraints.idxbu = np.array([0, 1, 2, 3])
        # -- Index for a1c, a2c, a3c, a4c
        if self.tilt:
            ocp.constraints.idxbu = np.append(ocp.constraints.idxbu, [4, 5, 6, 7])

        # -- Lower Input Bound
        ocp.constraints.lbu = np.array(
            [self.params["thrust_min"],
             self.params["thrust_min"],
             self.params["thrust_min"],
             self.params["thrust_min"]])

        if self.tilt:
            ocp.constraints.lbu = np.append(ocp.constraints.lbu,
                [self.params["a_min"],
                 self.params["a_min"],
                 self.params["a_min"],
                 self.params["a_min"]])

        # -- Upper Input Bound
        ocp.constraints.ubu = np.array(
            [self.params["thrust_max"],
             self.params["thrust_max"],
             self.params["thrust_max"],
             self.params["thrust_max"]])

        if self.tilt:
            ocp.constraints.ubu = np.append(ocp.constraints.ubu,
                [self.params["a_max"],
                 self.params["a_max"],
                 self.params["a_max"],
                 self.params["a_max"]])

        # Nonlinear constraint for quaternions
        if self.include_quaternion_constraint:
            # Note: This is necessary to ensure that the quaternion stays on the unit sphere
            # and represents a valid rotation.
            ocp.model.con_h_expr = self.qw ** 2 + self.qx ** 2 + self.qy ** 2 + self.qz ** 2 - 1.0   # ||q||^2 - 1 = 0
            ocp.model.con_h_expr_e = ocp.model.con_h_expr
            ocp.constraints.lh = np.array([0.0])
            ocp.constraints.uh = np.array([0.0])
            ocp.constraints.lh_e = np.array([0.0])
            ocp.constraints.uh_e = np.array([0.0])

        # - Slack variables for soft constraints
        if self.include_soft_constraints:
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
            ocp.cost.Zl =   self.params["linear_slack_weight"]*np.ones((num_constraints,))
            ocp.cost.Zu =   self.params["linear_slack_weight"]*np.ones((num_constraints,))
            ocp.cost.Zl_e = self.params["linear_slack_weight"]*np.ones((num_constraints_e,))
            ocp.cost.Zu_e = self.params["linear_slack_weight"]*np.ones((num_constraints_e,))
            # -- Quadratic term
            ocp.cost.zl =   self.params["quadratic_slack_weight"]*np.ones((num_constraints,))
            ocp.cost.zu =   self.params["quadratic_slack_weight"]*np.ones((num_constraints,))
            ocp.cost.zl_e = self.params["quadratic_slack_weight"]*np.ones((num_constraints_e,))
            ocp.cost.zu_e = self.params["quadratic_slack_weight"]*np.ones((num_constraints_e,))

        # Initial state and reference: Set all values such that robot is hovering
        x_ref = np.zeros(nx)
        x_ref[6] = 1.0  # Quaternion qw

        if self.tilt:
            # When included servo AND thrust, use further indices 
            if self.include_servo_model and self.include_thrust_model:
                x_ref[17:21] = self.phys.mass * self.phys.gravity / 4  # ft1s, ft2s, ft3s, ft4s
            # When only included thrust, use the same indices
            elif self.include_thrust_model:
                x_ref[13:17] = self.phys.mass * self.phys.gravity / 4  # ft1s, ft2s, ft3s, ft4s
        else:
            x_ref[13:17] = self.phys.mass * self.phys.gravity / 4  # ft1s, ft2s, ft3s, ft4s

        ocp.constraints.x0 = x_ref  # TODO this should be set in control loop and updated before each solver call

        # Note: This is not really necessary, since the reference is always updated before solver is called
        u_ref = np.zeros(nu)
        # Obeserved to be worse than zero!
        u_ref[0:4] = self.phys.mass * self.phys.gravity / 4  # ft1c, ft2c, ft3c, ft4c
        ocp.cost.yref = np.concatenate((x_ref, u_ref))
        ocp.cost.yref_e = x_ref

        # Model parameters
        # same order: phy_params = ca.vertcat(mass, gravity, inertia, kq_d_kt, dr, p1_b, p2_b, p3_b, p4_b, t_rotor, t_servo)
        self.acados_init_p = np.zeros(n_param)
        self.acados_init_p[0] = x_ref[6]  # qw
        if len(self.phys.physical_param_list) != 24:
            raise ValueError("Physical parameters are not in the correct order. Please check the physical model.")
        self.acados_init_p[4:28] = np.array(self.phys.physical_param_list)

        ocp.parameter_values = self.acados_init_p

        # Solver options
        ocp.solver_options.tf = self.params["T_horizon"]
        ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM" # "IPOPT", "FULL_CONDENSING_HPIPM"
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

    @abstractmethod
    def get_reference(self):
        pass

    def _create_reference_generator(self) -> QDNMPCReferenceGenerator:
        # Pass the model's and robot's properties to the reference generator
        return QDNMPCReferenceGenerator(self,
                                        self.phys.p1_b,    self.phys.p2_b, self.phys.p3_b, self.phys.p4_b,
                                        self.phys.dr1,     self.phys.dr2,  self.phys.dr3,  self.phys.dr4,
                                        self.phys.kq_d_kt, self.phys.mass, self.phys.gravity)

    def create_acados_sim_solver(self, ts_sim: float, build: bool = True) -> AcadosSimSolver:
        ocp_model = super().get_acados_model()

        acados_sim = AcadosSim()
        acados_sim.model = ocp_model

        n_param = ocp_model.p.size()[0]
        # same order: phy_params = ca.vertcat(mass, gravity, inertia, kq_d_kt, dr, p1_b, p2_b, p3_b, p4_b, t_rotor, t_servo)
        self.acados_init_p = np.zeros(n_param)
        self.acados_init_p[0] = 1.0  # qw
        self.acados_init_p[4:28] = np.array(self.phys.physical_param_list)
        acados_sim.parameter_values = self.acados_init_p

        acados_sim.solver_options.T = ts_sim
        return AcadosSimSolver(acados_sim, json_file=ocp_model.name + "_acados_sim.json", build=build)
