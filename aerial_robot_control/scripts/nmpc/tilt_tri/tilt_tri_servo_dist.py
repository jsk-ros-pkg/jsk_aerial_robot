#!/usr/bin/env python
# -*- encoding: ascii -*-
import os, sys
import numpy as np
import casadi as ca
from acados_template import AcadosModel, AcadosOcpSolver, AcadosSim, AcadosSimSolver

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))    # Add parent directory to path to allow relative imports
from tri_reference_generator import TriNMPCReferenceGenerator
from rh_base import RecedingHorizonBase

import phys_param_trirotor as phys


class NMPCTiltTriServoDist(RecedingHorizonBase):
    def __init__(self, overwrite: bool = False):
        # Model name
        self.model_name = "tilt_tri_servo_dist_mdl"

        # ====== Define controller setup through flags ======
        #
        # - tilt: Flag to include tiltable rotors and their effect on the rotation matrix.
        # - include_servo_model: Flag to include the servo model based on the angle alpha (a) between frame E (end of arm) and R (rotor). If not included, angle control is assumed to be equal to angle state.
        # - include_servo_derivative: Flag to include the continuous time-derivative of the servo angle as control input(!) instead of numeric differentation.
        # - include_thrust_model: Flag to include dynamics from rotor and use thrust as state. If not included, thrust control is assumed to be equal to thrust state.
        # - include_cog_dist_model: Flag to include disturbance on the CoG into the acados model states. Disturbance on each rotor individually was investigated into but didn't properly work, therefore only disturbance on CoG implemented.
        # - include_cog_dist_parameter: Flag to include disturbance on the CoG into the acados model parameters. Disturbance on each rotor individually was investigated into but didn't properly work, therefore only disturbance on CoG implemented.
        # - include_impedance: Flag to include virtual mass and inertia to calculate impedance cost. Doesn't add any functionality for the model.
        # - include_a_prev: Flag to include reference value for the servo angle command in NMPCReferenceGenerator() based on command from previous timestep.
        
        self.tilt = True
        self.include_servo_model = True
        self.include_servo_derivative = False
        self.include_thrust_model = False
        self.include_cog_dist_model = False
        self.include_cog_dist_parameter = False
        self.include_impedance = False

        # Load robot specific parameters
        self.phys = phys

        # Read parameters from configuration file in the robot's package
        self.read_params("controller", "nmpc", "gimbalrotor", "TiltTriRotorNMPC.yaml")
        
        # Create acados model & solver and generate c code
        super().__init__("nmpc", overwrite)

        # Create Reference Generator object
        self._reference_generator = self._create_reference_generator()

    def create_acados_model(self) -> AcadosModel:
        # Model states
        p = ca.SX.sym("p", 3)
        v = ca.SX.sym("v", 3)

        qw = ca.SX.sym("qw")
        qx = ca.SX.sym("qx")
        qy = ca.SX.sym("qy")
        qz = ca.SX.sym("qz")
        q = ca.vertcat(qw, qx, qy, qz)

        wx = ca.SX.sym("wx")
        wy = ca.SX.sym("wy")
        wz = ca.SX.sym("wz")
        w = ca.vertcat(wx, wy, wz)

        a1 = ca.SX.sym("a1")
        a2 = ca.SX.sym("a2")
        a3 = ca.SX.sym("a3")
        a = ca.vertcat(a1, a2, a3)

        states = ca.vertcat(p, v, q, w, a)

        # Model parameters
        qwr = ca.SX.sym("qwr")  # Reference for quaternions
        qxr = ca.SX.sym("qxr")
        qyr = ca.SX.sym("qyr")
        qzr = ca.SX.sym("qzr")
        quaternion = ca.vertcat(qwr, qxr, qyr, qzr)

        # CoG disturbance
        fdp_w = ca.SX.sym("fdp_w", 3)
        tau_dp_b = ca.SX.sym("tau_dp_b", 3)

        parameters = ca.vertcat(quaternion, fdp_w, tau_dp_b)

        # Control inputs
        ft1c = ca.SX.sym("ft1c")
        ft2c = ca.SX.sym("ft2c")
        ft3c = ca.SX.sym("ft3c")
        ft_c = ca.vertcat(ft1c, ft2c, ft3c)
        a1c = ca.SX.sym("a1c")
        a2c = ca.SX.sym("a2c")
        a3c = ca.SX.sym("a3c")
        a_c = ca.vertcat(a1c, a2c, a3c)
        controls = ca.vertcat(ft_c, a_c)

        # Transformation matrix
        row_1 = ca.horzcat(
            ca.SX(1 - 2 * qy**2 - 2 * qz**2), ca.SX(2 * qx * qy - 2 * qw * qz), ca.SX(2 * qx * qz + 2 * qw * qy)
        )
        row_2 = ca.horzcat(
            ca.SX(2 * qx * qy + 2 * qw * qz), ca.SX(1 - 2 * qx**2 - 2 * qz**2), ca.SX(2 * qy * qz - 2 * qw * qx)
        )
        row_3 = ca.horzcat(
            ca.SX(2 * qx * qz - 2 * qw * qy), ca.SX(2 * qy * qz + 2 * qw * qx), ca.SX(1 - 2 * qx**2 - 2 * qy**2)
        )
        rot_ib = ca.vertcat(row_1, row_2, row_3)

        den = np.sqrt(self.phys.p1_b[0] ** 2 + self.phys.p1_b[1] ** 2)
        rot_be1 = np.array([[self.phys.p1_b[0] / den, -self.phys.p1_b[1] / den, 0], [self.phys.p1_b[1] / den, self.phys.p1_b[0] / den, 0], [0, 0, 1]])

        den = np.sqrt(self.phys.p2_b[0] ** 2 + self.phys.p2_b[1] ** 2)
        rot_be2 = np.array([[self.phys.p2_b[0] / den, -self.phys.p2_b[1] / den, 0], [self.phys.p2_b[1] / den, self.phys.p2_b[0] / den, 0], [0, 0, 1]])

        den = np.sqrt(self.phys.p3_b[0] ** 2 + self.phys.p3_b[1] ** 2)
        rot_be3 = np.array([[self.phys.p3_b[0] / den, -self.phys.p3_b[1] / den, 0], [self.phys.p3_b[1] / den, self.phys.p3_b[0] / den, 0], [0, 0, 1]])

        rot_e1r1 = ca.vertcat(
            ca.horzcat(1, 0, 0), ca.horzcat(0, ca.cos(a1), -ca.sin(a1)), ca.horzcat(0, ca.sin(a1), ca.cos(a1))
        )
        rot_e2r2 = ca.vertcat(
            ca.horzcat(1, 0, 0), ca.horzcat(0, ca.cos(a2), -ca.sin(a2)), ca.horzcat(0, ca.sin(a2), ca.cos(a2))
        )
        rot_e3r3 = ca.vertcat(
            ca.horzcat(1, 0, 0), ca.horzcat(0, ca.cos(a3), -ca.sin(a3)), ca.horzcat(0, ca.sin(a3), ca.cos(a3))
        )

        # Wrench in Rotor frame
        ft_r1 = ca.vertcat(0, 0, ft1c)
        ft_r2 = ca.vertcat(0, 0, ft2c)
        ft_r3 = ca.vertcat(0, 0, ft3c)

        tau_r1 = ca.vertcat(0, 0, -self.phys.dr1 * ft1c * self.phys.kq_d_kt)
        tau_r2 = ca.vertcat(0, 0, -self.phys.dr2 * ft2c * self.phys.kq_d_kt)
        tau_r3 = ca.vertcat(0, 0, -self.phys.dr3 * ft3c * self.phys.kq_d_kt)

        # Wrench in Body frame
        f_u_b = (
              ca.mtimes(rot_be1, ca.mtimes(rot_e1r1, ft_r1))
            + ca.mtimes(rot_be2, ca.mtimes(rot_e2r2, ft_r2))
            + ca.mtimes(rot_be3, ca.mtimes(rot_e3r3, ft_r3))
        )
        tau_u_b = (
              ca.mtimes(rot_be1, ca.mtimes(rot_e1r1, tau_r1))
            + ca.mtimes(rot_be2, ca.mtimes(rot_e2r2, tau_r2))
            + ca.mtimes(rot_be3, ca.mtimes(rot_e3r3, tau_r3))
            + ca.cross(np.array(self.phys.p1_b), ca.mtimes(rot_be1, ca.mtimes(rot_e1r1, ft_r1)))
            + ca.cross(np.array(self.phys.p2_b), ca.mtimes(rot_be2, ca.mtimes(rot_e2r2, ft_r2)))
            + ca.cross(np.array(self.phys.p3_b), ca.mtimes(rot_be3, ca.mtimes(rot_e3r3, ft_r3)))
        )

        # Inertia
        I = ca.diag([self.phys.Ixx, self.phys.Iyy, self.phys.Izz])
        I_inv = ca.diag([1 / self.phys.Ixx, 1 / self.phys.Iyy, 1 / self.phys.Izz])
        g_i = np.array([0, 0, -self.phys.gravity])

        # dynamic model
        ds = ca.vertcat(
            v,
            ca.mtimes(rot_ib, f_u_b) / self.phys.mass + g_i + fdp_w / self.phys.mass,
            (-wx * qx - wy * qy - wz * qz) / 2,
            (wx * qw + wz * qy - wy * qz) / 2,
            (wy * qw - wz * qx + wx * qz) / 2,
            (wz * qw + wy * qx - wx * qy) / 2,
            ca.mtimes(I_inv, (-ca.cross(w, ca.mtimes(I, w)) + tau_u_b + tau_dp_b)),
            (a_c - a) / self.phys.t_servo,
        )

        # Assemble acados function
        f = ca.Function("f", [states, controls], [ds], ["state", "control_input"], ["ds"], {"allow_free": True})

        # Implicit dynamics
        x_dot = ca.SX.sym("x_dot", states.size())
        f_impl = x_dot - f(states, controls)

        # Cost function
        # error = y - y_ref
        # NONLINEAR_LS = error^T @ Q @ error
        # qe = qr^* multiply q
        qe_x = qwr * qx - qw * qxr - qyr * qz + qy * qzr
        qe_y = qwr * qy - qw * qyr + qxr * qz - qx * qzr
        qe_z = -qxr * qy + qx * qyr + qwr * qz - qw * qzr

        state_y = ca.vertcat(p, v, qwr, qe_x + qxr, qe_y + qyr, qe_z + qzr, w, a)
        state_y_e = state_y
        control_y = ca.vertcat(ft_c, (a_c - a))  # ac_ref must be zero!

        # Assemble acados model
        model = AcadosModel()
        model.name = self.model_name
        model.f_expl_expr = f(states, controls)  # CasADi expression for the explicit dynamics
        model.f_impl_expr = f_impl  # CasADi expression for the implicit dynamics
        model.x = states
        model.xdot = x_dot
        model.u = controls
        model.p = parameters
        model.cost_y_expr = ca.vertcat(state_y, control_y)  # NONLINEAR_LS
        model.cost_y_expr_e = state_y_e

        return model

    def create_acados_ocp_solver(self) -> AcadosOcpSolver:
        # Get OCP object
        ocp = super().get_ocp()

        # Model dimensions
        nx = ocp.model.x.size()[0]; nu = ocp.model.u.size()[0]

        # Define weights
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
            ]
        )
        print("Q: \n", Q)

        R = np.diag(
            [
                self.params["Rt"],
                self.params["Rt"],
                self.params["Rt"],
                self.params["Rac_d"],
                self.params["Rac_d"],
                self.params["Rac_d"],
            ]
        )
        print("R: \n", R)

        # Cost function options
        ocp.cost.cost_type = "NONLINEAR_LS"
        ocp.cost.cost_type_e = "NONLINEAR_LS"
        ocp.cost.W = np.block([[Q, np.zeros((nx, nu))], [np.zeros((nu, nx)), R]])
        ocp.cost.W_e = Q  # weight matrix at terminal shooting node (N).

        # set constraints
        # - State box constraints bx
        # vx, vy, vz, wx, wy, wz, a1, a2, a3
        ocp.constraints.idxbx = np.array([3, 4, 5, 10, 11, 12, 13, 14, 15])
        ocp.constraints.lbx = np.array(
            [
                self.params["v_min"],
                self.params["v_min"],
                self.params["v_min"],
                self.params["w_min"],
                self.params["w_min"],
                self.params["w_min"],
                self.params["a_min"],
                self.params["a_min"],
                self.params["a_min"],
            ]
        )
        ocp.constraints.ubx = np.array(
            [
                self.params["v_max"],
                self.params["v_max"],
                self.params["v_max"],
                self.params["w_max"],
                self.params["w_max"],
                self.params["w_max"],
                self.params["a_max"],
                self.params["a_max"],
                self.params["a_max"],
            ]
        )
        print("lbx: ", ocp.constraints.lbx)
        print("ubx: ", ocp.constraints.ubx)

        # - Terminal state box constraints bx_e
        # vx, vy, vz, wx, wy, wz, a1, a2, a3
        ocp.constraints.idxbx_e = np.array([3, 4, 5, 10, 11, 12, 13, 14, 15])
        ocp.constraints.lbx_e = np.array(
            [
                self.params["v_min"],
                self.params["v_min"],
                self.params["v_min"],
                self.params["w_min"],
                self.params["w_min"],
                self.params["w_min"],
                self.params["a_min"],
                self.params["a_min"],
                self.params["a_min"],
            ]
        )
        ocp.constraints.ubx_e = np.array(
            [
                self.params["v_max"],
                self.params["v_max"],
                self.params["v_max"],
                self.params["w_max"],
                self.params["w_max"],
                self.params["w_max"],
                self.params["a_max"],
                self.params["a_max"],
                self.params["a_max"],
            ]
        )
        print("lbx_e: ", ocp.constraints.lbx_e)
        print("ubx_e: ", ocp.constraints.ubx_e)

        # - Input box constraints bu
        # ft1, ft2, ft3, a1c, a2c, a3c
        ocp.constraints.idxbu = np.array([0, 1, 2, 3, 4, 5])
        ocp.constraints.lbu = np.array(
            [
                self.params["thrust_min"],
                self.params["thrust_min"],
                self.params["thrust_min"],
                self.params["a_min"],
                self.params["a_min"],
                self.params["a_min"],
            ]
        )
        ocp.constraints.ubu = np.array(
            [
                self.params["thrust_max"],
                self.params["thrust_max"],
                self.params["thrust_max"],
                self.params["a_max"],
                self.params["a_max"],
                self.params["a_max"],
            ]
        )
        print("lbu: ", ocp.constraints.lbu)
        print("ubu: ", ocp.constraints.ubu)

        # Initial state and reference
        x_ref = np.zeros(nx)
        x_ref[6] = 1.0  # qw
        u_ref = np.zeros(nu)
        u_ref[0:3] = self.phys.mass * self.phys.gravity / 3  # ft1, ft2, ft3
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

        # Compile acados OCP
        json_file_path = os.path.join("./" + ocp.model.name + "_acados_ocp.json")
        solver = AcadosOcpSolver(ocp, json_file=json_file_path, build=True)
        print("Generated C code for acados solver successfully to " + os.getcwd())

        return solver

    def get_reference(self, target_xyz, target_qwxyz, ft_ref, a_ref):
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
        ocp = self.get_ocp(); nn = ocp.dims.N
        nx = ocp.dims.nx; nu = ocp.dims.nu

        # Assemble state reference
        xr = np.zeros([nn + 1, nx])
        xr[:, 0] = target_xyz[0]       # x
        xr[:, 1] = target_xyz[1]       # y
        xr[:, 2] = target_xyz[2]       # z
        # No reference for vx, vy, vz (idx: 3, 4, 5)
        xr[:, 6] = target_qwxyz[0]     # qx
        xr[:, 7] = target_qwxyz[1]     # qx
        xr[:, 8] = target_qwxyz[2]     # qy
        xr[:, 9] = target_qwxyz[3]     # qz
        # No reference for wx, wy, wz (idx: 10, 11, 12)
        xr[:, 13] = a_ref[0]
        xr[:, 14] = a_ref[1]
        xr[:, 15] = a_ref[2]

        # Assemble input reference
        # Note: Reference has to be zero if variable is included as state in cost function!
        ur = np.zeros([nn, nu])
        ur[:, 0] = ft_ref[0]
        ur[:, 1] = ft_ref[1]
        ur[:, 2] = ft_ref[2]

        return xr, ur
            
    def get_reference_generator(self) -> TriNMPCReferenceGenerator:
        return self._reference_generator
    
    def _create_reference_generator(self) -> TriNMPCReferenceGenerator:
        # Pass the model's and robot's properties to the reference generator
        return TriNMPCReferenceGenerator(self,
                                         self.phys.p1_b,    self.phys.p2_b, self.phys.p3_b,
                                         self.phys.dr1,     self.phys.dr2,  self.phys.dr3,
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


if __name__ == "__main__":
    overwrite = True
    nmpc = NMPCTiltTriServoDist(overwrite)

    acados_ocp_solver = nmpc.get_ocp_solver()
    print("Successfully initialized acados ocp: ", acados_ocp_solver.acados_ocp)
    print("number of states: ", acados_ocp_solver.acados_ocp.dims.nx)
    print("number of controls: ", acados_ocp_solver.acados_ocp.dims.nu)
    print("number of parameters: ", acados_ocp_solver.acados_ocp.dims.np)
    print("T_samp: ", nmpc.params["T_samp"])
    print("T_horizon: ", nmpc.params["T_horizon"])
    print("T_step: ", nmpc.params["T_step"])
    print("N_steps: ", nmpc.params["N_steps"])
