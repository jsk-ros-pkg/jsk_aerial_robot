import os, sys
from abc import abstractmethod
import numpy as np
from acados_template import AcadosModel, AcadosOcpSolver, AcadosSim, AcadosSimSolver
import casadi as ca

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))    # Add parent directory to path to allow relative imports
from rh_base import RecedingHorizonBase
from qd_reference_generator import QDNMPCReferenceGenerator

import phys_param_beetle_omni as phys


class QDNMPCBase(RecedingHorizonBase):
    """
    Base class for all NMPC controllers for quadrotors.
    Inherits from RecedingHorizonBase which also lays foundations for MHE classes.
    The child classes only have specifications which define the controller specifications and need to set the following flags:
     - include_servo_model: Flag to include the servo model based on the angle alpha (a) between frame E (end of arm) and R (rotor). If not included, angle control is assumed to be equal to angle state.
     - include_servo_derivative: Flag to include the continuous time-derivative of the servo angle as control input(!) instead of numeric differentation.
     - include_thrust_model: Flag to include dynamics from rotor and use thrust as state. If not included, thrust control is assumed to be equal to thrust state.
     - include_cog_dist_model: Flag to include disturbance on the CoG into the acados model states. Disturbance on each rotor individually was investigated into but didn't properly work, therefore only disturbance on CoG implemented.
     - include_cog_dist_parameter: Flag to include disturbance on the CoG into the acados model parameters. Disturbance on each rotor individually was investigated into but didn't properly work, therefore only disturbance on CoG implemented.
     - include_impedance: Flag to include virtual mass and inertia to calculate impedance cost. Doesn't add any functionality for the model.
     - include_a_prev: Flag to include reference value for the servo angle command in NMPCReferenceGenerator() based on command from previous timestep.

    :param str model_name: Name of the model defined in controller file.
    :param bool overwrite: Flag to overwrite existing c generated code for the OCP solver. Default: False
    """
    def __init__(self, model_name: str, overwrite: bool = False):
        # Store model name
        self.model_name = model_name

        # Load robot specific parameters
        self.phys = phys

        # Call RecedingHorizon constructor coming as NMPC method
        super().__init__("nmpc", overwrite)

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
        if self.include_servo_derivative and not self.include_servo_model: raise ValueError("Servo derivative can only work with servo angle defined as state through the 'include_servo_model' flag.")
        phys = self.phys

        # Standard state-space (Note: store in self to access for cost function in child controller class)
        self.x = ca.SX.sym("x")      # Position
        self.y = ca.SX.sym("y")
        self.z = ca.SX.sym("z"); self.p = ca.vertcat(self.x, self.y, self.z)
        self.vx = ca.SX.sym("vx")    # Linear velocity
        self.vy = ca.SX.sym("vy")
        self.vz = ca.SX.sym("vz"); self.v = ca.vertcat(self.vx, self.vy, self.vz)
        self.qw = ca.SX.sym("qw")    # Quaternions
        self.qx = ca.SX.sym("qx")
        self.qy = ca.SX.sym("qy")
        self.qz = ca.SX.sym("qz"); self.q = ca.vertcat(self.qw, self.qx, self.qy, self.qz)
        self.wx = ca.SX.sym("wx")    # Angular velocity
        self.wy = ca.SX.sym("wy")
        self.wz = ca.SX.sym("wz"); self.w = ca.vertcat(self.wx, self.wy, self.wz)
        states = ca.vertcat(self.p, self.v, self.q, self.w)
        
        # - Extend state-space by dynamics of servo angles (actual)
        # Differentiate between actual angles and control angles
        # Note: If servo angle is not used as control input the model for omnidirectional Quadrotor
        # has been observed to be unstable (see https://arxiv.org/abs/2405.09871).
        if self.tilt and self.include_servo_model:
            self.a1s = ca.SX.sym("a1s")
            self.a2s = ca.SX.sym("a2s")
            self.a3s = ca.SX.sym("a3s")
            self.a4s = ca.SX.sym("a4s")
            self.a_s = ca.vertcat(self.a1s, self.a2s, self.a3s, self.a4s)
            states = ca.vertcat(states, self.a_s)

        # - Extend state-space by dynamics of rotor (actual)
        # Differentiate between actual thrust and control thrust
        if self.include_thrust_model:
            self.ft1s = ca.SX.sym("ft1s")
            self.ft2s = ca.SX.sym("ft2s")
            self.ft3s = ca.SX.sym("ft3s")
            self.ft4s = ca.SX.sym("ft4s")
            self.ft_s = ca.vertcat(self.ft1s, self.ft2s, self.ft3s, self.ft4s)
            states = ca.vertcat(states, self.ft_s)
        
        # - Extend state-space by disturbance on CoG (actual)
        # Differentiate between actual disturbance set as state and set as parameter
        if self.include_cog_dist_model:
            # Force disturbance applied to CoG in World frame
            self.fds_w = ca.SX.sym("fds_w", 3)
            # Torque disturbance applied to CoG in Body frame
            self.tau_ds_b = ca.SX.sym("tau_ds_b", 3)

            states = ca.vertcat(states, self.fds_w, self.tau_ds_b)
        else:
            self.fds_w = ca.vertcat(0.0, 0.0, 0.0); self.tau_ds_b = ca.vertcat(0.0, 0.0, 0.0)

        # Control inputs
        # - Forces from thrust at each rotor
        self.ft1c = ca.SX.sym("ft1c")
        self.ft2c = ca.SX.sym("ft2c")
        self.ft3c = ca.SX.sym("ft3c")
        self.ft4c = ca.SX.sym("ft4c")
        self.ft_c = ca.vertcat(self.ft1c, self.ft2c, self.ft3c, self.ft4c)
        controls = ca.vertcat(self.ft1c, self.ft2c, self.ft3c, self.ft4c)
        # - Servo angle for tiltable rotors (actuated)
        if self.tilt:
            self.a1c = ca.SX.sym("a1c")
            self.a2c = ca.SX.sym("a2c")
            self.a3c = ca.SX.sym("a3c")
            self.a4c = ca.SX.sym("a4c")
            # Either use the time-derivative of the servo angle as control input directly
            if self.include_servo_derivative:
                self.ad_c = ca.vertcat(self.a1c, self.a2c, self.a3c, self.a4c)
                controls = ca.vertcat(controls, self.ad_c)
            # Or use numerical differentation to calculate time-derivate in dynamical model
            else:
                self.a_c = ca.vertcat(self.a1c, self.a2c, self.a3c, self.a4c)
                controls = ca.vertcat(controls, self.a_c)

        # Model parameters
        self.qwr = ca.SX.sym("qwr")  # Reference for quaternions
        self.qxr = ca.SX.sym("qxr")
        self.qyr = ca.SX.sym("qyr")
        self.qzr = ca.SX.sym("qzr")
        parameters = ca.vertcat(self.qwr, self.qxr, self.qyr, self.qzr)
        
        # - Extend model parameters by CoG disturbance
        if self.include_cog_dist_parameter:
            # Force disturbance applied to CoG in World frame
            self.fdp_w = ca.SX.sym("fdp_w", 3)
            # Torque disturbance applied to CoG in Body frame
            self.tau_dp_b = ca.SX.sym("tau_dp_b", 3)

            parameters = ca.vertcat(parameters, self.fdp_w, self.tau_dp_b)
        else:
            self.fdp_w = ca.vertcat(0.0, 0.0, 0.0); self.tau_dp_b = ca.vertcat(0.0, 0.0, 0.0)

        # - Extend model parameters by virtual mass and inertia for impedance cost function
        if self.include_impedance:
            if not self.include_cog_dist_model or not self.include_cog_dist_parameter: raise ValueError("Impedance cost can only be calculated if disturbance flags are activated.")

            self.mpx = ca.SX.sym("mpx")      # Virtual mass (p = position)
            self.mpy = ca.SX.sym("mpy")
            self.mpz = ca.SX.sym("mpz")
            mp = ca.vertcat(self.mpx, self.mpy, self.mpz)

            self.mqx = ca.SX.sym("mqx")      # Virtual inertia (q = quaternion)
            self.mqy = ca.SX.sym("mqy")
            self.mqz = ca.SX.sym("mqz")
            mq = ca.vertcat(self.mqx, self.mqy, self.mqz)

            parameters = ca.vertcat(parameters, mp, mq)

        # Transformation matrices between coordinate systems World, Body, End-of-arm, Rotor using quaternions
        # - World to Body
        row_1 = ca.horzcat(
            ca.SX(1 - 2 * self.qy**2 - 2 * self.qz**2), ca.SX(2 * self.qx * self.qy - 2 * self.qw * self.qz), ca.SX(2 * self.qx * self.qz + 2 * self.qw * self.qy)
        )
        row_2 = ca.horzcat(
            ca.SX(2 * self.qx * self.qy + 2 * self.qw * self.qz), ca.SX(1 - 2 * self.qx**2 - 2 * self.qz**2), ca.SX(2 * self.qy * self.qz - 2 * self.qw * self.qx)
        )
        row_3 = ca.horzcat(
            ca.SX(2 * self.qx * self.qz - 2 * self.qw * self.qy), ca.SX(2 * self.qy * self.qz + 2 * self.qw * self.qx), ca.SX(1 - 2 * self.qx**2 - 2 * self.qy**2)
        )
        rot_wb = ca.vertcat(row_1, row_2, row_3)    
        # - Body to End-of-arm
        denominator = np.sqrt(phys.p1_b[0] ** 2 + phys.p1_b[1] ** 2)
        rot_be1 = np.array([[phys.p1_b[0] / denominator, -phys.p1_b[1] / denominator, 0], [phys.p1_b[1] / denominator, phys.p1_b[0] / denominator, 0], [0, 0, 1]])

        denominator = np.sqrt(phys.p2_b[0] ** 2 + phys.p2_b[1] ** 2)
        rot_be2 = np.array([[phys.p2_b[0] / denominator, -phys.p2_b[1] / denominator, 0], [phys.p2_b[1] / denominator, phys.p2_b[0] / denominator, 0], [0, 0, 1]])

        denominator = np.sqrt(phys.p3_b[0] ** 2 + phys.p3_b[1] ** 2)
        rot_be3 = np.array([[phys.p3_b[0] / denominator, -phys.p3_b[1] / denominator, 0], [phys.p3_b[1] / denominator, phys.p3_b[0] / denominator, 0], [0, 0, 1]])

        denominator = np.sqrt(phys.p4_b[0] ** 2 + phys.p4_b[1] ** 2)
        rot_be4 = np.array([[phys.p4_b[0] / denominator, -phys.p4_b[1] / denominator, 0], [phys.p4_b[1] / denominator, phys.p4_b[0] / denominator, 0], [0, 0, 1]])

        # - End-of-arm to Rotor
        # Take tilt rotation with angle alpha (a) of R frame to E frame into account
        if self.tilt:
            # If servo dynamics are modeled, use angle state.
            # Else use angle control which is then assumed to be equal to the angle state at all times.
            if self.include_servo_model: 
                a1 = self.a1s; a2 = self.a2s; a3 = self.a3s; a4 = self.a4s
            else:
                a1 = self.a1c; a2 = self.a2c; a3 = self.a3c; a4 = self.a4c

            rot_e1r1 = ca.vertcat(
                ca.horzcat(1, 0, 0), ca.horzcat(0, ca.cos(a1), -ca.sin(a1)), ca.horzcat(0, ca.sin(a1), ca.cos(a1))
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
            rot_e1r1 = ca.SX.eye(3); rot_e2r2 = ca.SX.eye(3); rot_e3r3 = ca.SX.eye(3); rot_e4r4 = ca.SX.eye(3)

        # Wrench in Rotor frame
        # If rotor dynamics are modeled, explicitly use thrust state as force.
        # Else use thrust control which is then assumed to be equal to the thrust state at all times.
        if self.include_thrust_model:
            ft1 = self.ft1s; ft2 = self.ft2s; ft3 = self.ft3s; ft4 = self.ft4s
        else:
            ft1 = self.ft1c; ft2 = self.ft2c; ft3 = self.ft3c; ft4 = self.ft4c
        
        ft_r1 = ca.vertcat(0, 0, ft1)
        ft_r2 = ca.vertcat(0, 0, ft2)
        ft_r3 = ca.vertcat(0, 0, ft3)
        ft_r4 = ca.vertcat(0, 0, ft4)

        tau_r1 = ca.vertcat(0, 0, -phys.dr1 * ft1 * phys.kq_d_kt)
        tau_r2 = ca.vertcat(0, 0, -phys.dr2 * ft2 * phys.kq_d_kt)
        tau_r3 = ca.vertcat(0, 0, -phys.dr3 * ft3 * phys.kq_d_kt)
        tau_r4 = ca.vertcat(0, 0, -phys.dr4 * ft4 * phys.kq_d_kt)

        # Wrench in Body frame
        fu_b = (
              ca.mtimes(rot_be1, ca.mtimes(rot_e1r1, ft_r1))
            + ca.mtimes(rot_be2, ca.mtimes(rot_e2r2, ft_r2))
            + ca.mtimes(rot_be3, ca.mtimes(rot_e3r3, ft_r3))
            + ca.mtimes(rot_be4, ca.mtimes(rot_e4r4, ft_r4))
        )
        tau_u_b = (
              ca.mtimes(rot_be1, ca.mtimes(rot_e1r1, tau_r1))
            + ca.mtimes(rot_be2, ca.mtimes(rot_e2r2, tau_r2))
            + ca.mtimes(rot_be3, ca.mtimes(rot_e3r3, tau_r3))
            + ca.mtimes(rot_be4, ca.mtimes(rot_e4r4, tau_r4))
            + ca.cross(np.array(phys.p1_b), ca.mtimes(rot_be1, ca.mtimes(rot_e1r1, ft_r1)))
            + ca.cross(np.array(phys.p2_b), ca.mtimes(rot_be2, ca.mtimes(rot_e2r2, ft_r2)))
            + ca.cross(np.array(phys.p3_b), ca.mtimes(rot_be3, ca.mtimes(rot_e3r3, ft_r3)))
            + ca.cross(np.array(phys.p4_b), ca.mtimes(rot_be4, ca.mtimes(rot_e4r4, ft_r4)))
        )

        # Compute Inertia
        I = ca.diag([phys.Ixx, phys.Iyy, phys.Izz])
        I_inv = ca.diag([1 / phys.Ixx, 1 / phys.Iyy, 1 / phys.Izz])
        g_w = np.array([0, 0, -phys.gravity])     # World frame

        # Dynamic model (Time-derivative of states)
        ds = ca.vertcat(
            self.v,
            (ca.mtimes(rot_wb, fu_b) + self.fds_w + self.fdp_w) / phys.mass + g_w,
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
                (self.a_c - self.a_s) / phys.t_servo    # Time constant of servo motor
            )
            
        # - Extend model by thrust first-order dynamics
        # Assumption if not included: f_tc = f_ts
        if self.include_thrust_model:
            ds = ca.vertcat(ds,
                (self.ft_c - self.ft_s) / phys.t_rotor  # Time constant of rotor
            )
        
        # - Extend model by disturbances simply to match state dimensions
        if self.include_cog_dist_model:
            ds = ca.vertcat(ds,
                ca.vertcat(0.0, 0.0, 0.0),
                ca.vertcat(0.0, 0.0, 0.0),
            )
            
        # Assemble acados function
        f = ca.Function("f", [states, controls], [ds], ["state", "control_input"], ["ds"], {"allow_free": True})

        # Implicit dynamics
        # Note: Used only mainly because of acados template
        x_dot = ca.SX.sym("x_dot", states.size())      # Combined state vector 
        f_impl = x_dot - f(states, controls)

        # Get terms of cost function
        if self.include_impedance:
            # Compute linear acceleration (in World frame) and angular acceleration (in Body frame) for impedance cost
            # TODO clarify note and fix if necessary        
            # Note that this part should be f_d_i and no f_d_i_para, since the impedance should not respond to the I Term force.
            lin_acc_w = (ca.mtimes(rot_wb, fu_b) + self.fds_w + self.fdp_w) / phys.mass + g_w
            ang_acc_b = ca.mtimes(I_inv, (-ca.cross(self.w, ca.mtimes(I, self.w)) + tau_u_b + self.tau_ds_b + self.tau_dp_b))
            
            state_y, state_y_e, control_y = self.get_cost_function(lin_acc_w=lin_acc_w, ang_acc_b=ang_acc_b)
        else:
            state_y, state_y_e, control_y = self.get_cost_function()

        # Assemble acados model
        model = AcadosModel()
        model.name = self.model_name
        model.f_expl_expr = f(states, controls)  # CasADi expression for the explicit dynamics
        model.f_impl_expr = f_impl               # CasADi expression for the implicit dynamics
        model.x = states
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

    def create_acados_ocp_solver(self) -> AcadosOcpSolver:
        """
        Create generic acados solver for NMPC framework of a quadrotor.
        Generate c code into source folder in aerial_robot_control to be used in workflow.
        """
        # Get OCP object
        ocp = super().get_ocp()
        
        # Model dimensions
        nx = ocp.model.x.size()[0]; nu = ocp.model.u.size()[0]

        # Get weights from parametrization child file
        Q, R = self.get_weights()

        # Cost function options
        # see https://docs.acados.org/python_interface/#acados_template.acados_ocp_cost.AcadosOcpCost for details
        ocp.cost.cost_type = "NONLINEAR_LS"
        ocp.cost.cost_type_e = "NONLINEAR_LS"
        ocp.cost.W = np.block([[Q, np.zeros((nx, nu))], [np.zeros((nu, nx)), R]])
        ocp.cost.W_e = Q    # Weight matrix at terminal shooting node (N)

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
            [   self.params["v_min"],
                self.params["v_min"],
                self.params["v_min"],
                self.params["w_min"],
                self.params["w_min"],
                self.params["w_min"]     ])
        
        if self.tilt and self.include_servo_model:
            ocp.constraints.lbx = np.append(ocp.constraints.lbx,
            [   self.params["a_min"],
                self.params["a_min"],
                self.params["a_min"],
                self.params["a_min"]     ])

        if self.include_thrust_model:
            ocp.constraints.lbx = np.append(ocp.constraints.lbx,
            [   self.params["thrust_min"],
                self.params["thrust_min"],
                self.params["thrust_min"],
                self.params["thrust_min"]])

        # -- Upper State Bound
        ocp.constraints.ubx = np.array(
            [   self.params["v_max"],
                self.params["v_max"],
                self.params["v_max"],
                self.params["w_max"],
                self.params["w_max"],
                self.params["w_max"]     ])
        
        if self.tilt and self.include_servo_model:
            ocp.constraints.ubx = np.append(ocp.constraints.ubx,
            [   self.params["a_max"],
                self.params["a_max"],
                self.params["a_max"],
                self.params["a_max"]     ])

        if self.include_thrust_model:
            ocp.constraints.ubx = np.append(ocp.constraints.ubx,
            [   self.params["thrust_max"],
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
            [   self.params["v_min"],
                self.params["v_min"],
                self.params["v_min"],
                self.params["w_min"],
                self.params["w_min"],
                self.params["w_min"]     ])
        
        if self.tilt and self.include_servo_model:
            ocp.constraints.lbx_e = np.append(ocp.constraints.lbx_e,
            [   self.params["a_min"],
                self.params["a_min"],
                self.params["a_min"],
                self.params["a_min"]     ])
            
        if self.include_thrust_model:
            ocp.constraints.lbx_e = np.append(ocp.constraints.lbx_e,
            [   self.params["thrust_min"],
                self.params["thrust_min"],
                self.params["thrust_min"],
                self.params["thrust_min"]])

        # -- Upper Terminal State Bound
        ocp.constraints.ubx_e = np.array(
            [   self.params["v_max"],
                self.params["v_max"],
                self.params["v_max"],
                self.params["w_max"],
                self.params["w_max"],
                self.params["w_max"]     ])
        
        if self.tilt and self.include_servo_model:
            ocp.constraints.ubx_e = np.append(ocp.constraints.ubx_e,
            [   self.params["a_max"],
                self.params["a_max"],
                self.params["a_max"],
                self.params["a_max"]     ])
            
        if self.include_thrust_model:
            ocp.constraints.ubx_e = np.append(ocp.constraints.ubx_e,
            [   self.params["thrust_max"],
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
            [   self.params["thrust_min"],
                self.params["thrust_min"],
                self.params["thrust_min"],
                self.params["thrust_min"]])
        
        if self.tilt:
            ocp.constraints.lbu = np.append(ocp.constraints.lbu,
            [   self.params["a_min"],
                self.params["a_min"],
                self.params["a_min"],
                self.params["a_min"]     ])

        # -- Upper Input Bound
        ocp.constraints.ubu = np.array(
            [   self.params["thrust_max"],
                self.params["thrust_max"],
                self.params["thrust_max"],
                self.params["thrust_max"]])

        if self.tilt:
            ocp.constraints.ubu = np.append(ocp.constraints.ubu,
            [   self.params["a_max"],
                self.params["a_max"],
                self.params["a_max"],
                self.params["a_max"]     ])

        # Initial state and reference: Set all values such that robot is hovering
        # TODO debatable which initial states/inputs make sense -> not necessarily better than just all-zero!
        x_ref = np.zeros(nx)
        if self.tilt:
            x_ref[6] = 1.0      # Quaternion qw
            # When included servo AND thrust, use further indices 
            if self.include_servo_model and self.include_thrust_model:
                x_ref[17:21] = self.phys.mass * self.phys.gravity / 4       # ft1s, ft2s, ft3s, ft4s
            # When only included thrust, use the same indices
            elif self.include_thrust_model:
                x_ref[13:17] = self.phys.mass * self.phys.gravity / 4       # ft1s, ft2s, ft3s, ft4s
        else:
            x_ref[13:17] = self.phys.mass * self.phys.gravity / 4       # ft1s, ft2s, ft3s, ft4s
        u_ref = np.zeros(nu)
        # Obeserved to be worse than zero!
        u_ref[0:4] = self.phys.mass * self.phys.gravity / 4             # ft1c, ft2c, ft3c, ft4c
        
        ocp.constraints.x0 = x_ref
        ocp.cost.yref = np.concatenate((x_ref, u_ref))
        ocp.cost.yref_e = x_ref

        # Solver options
        ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
        ocp.solver_options.hpipm_mode = "BALANCE"  # "BALANCE", "SPEED_ABS", "SPEED", "ROBUST". Default: "BALANCE".
        # Start up flags:       [Seems only works for FULL_CONDENSING_QPOASES]
        # 0: no warm start; 1: warm start; 2: hot start. Default: 0
        # ocp.solver_options.qp_solver_warm_start = 1
        ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
        ocp.solver_options.integrator_type = "ERK"  # explicit Runge-Kutta integrator
        ocp.solver_options.print_level = 0
        ocp.solver_options.nlp_solver_type = "SQP_RTI"
        ocp.solver_options.qp_solver_cond_N = self.params["N_steps"]
        ocp.solver_options.tf = self.params["T_horizon"]

        # Build acados ocp into current working directory (which was created in super class)
        json_file_path = os.path.join("./" + ocp.model.name + "_acados_ocp.json")
        solver = AcadosOcpSolver(ocp, json_file=json_file_path, build=True)
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

    def create_acados_sim_solver(self, ts_sim: float, is_build: bool = True) -> AcadosSimSolver:
        ocp_model = super().get_acados_model()

        acados_sim = AcadosSim()
        acados_sim.model = ocp_model

        n_params = ocp_model.p.size()[0]
        acados_sim.dims.np = n_params
        acados_sim.parameter_values = np.zeros(n_params)

        acados_sim.solver_options.T = ts_sim
        return AcadosSimSolver(acados_sim, json_file=ocp_model.name + "_acados_sim.json", build=is_build)
