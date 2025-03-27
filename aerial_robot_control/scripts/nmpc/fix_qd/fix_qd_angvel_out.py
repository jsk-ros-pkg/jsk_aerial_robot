#!/usr/bin/env python
# -*- encoding: ascii -*-
import os, sys
import numpy as np
from acados_template import AcadosModel, AcadosOcpSolver, AcadosSim, AcadosSimSolver
import casadi as ca
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))    # Add parent directory to path to allow relative imports
from rh_base import RecedingHorizonBase
from tilt_qd.qd_reference_generator import QDNMPCReferenceGenerator

import archive.phys_param_beetle_art as phys     # Define physical parameters


class NMPCFixQdAngvelOut(RecedingHorizonBase):
    """
    Controller Name: Fixed Quadrotor NMPC with Angular Velocity Output
    This NMPC controller is used for a fixed-quadrotor (meaning the rotors are fixed to the body frame).
    The output of the controller is the body rate and collective acceleration.

    For information: The reason why this file doesnt get refactored to 'qd_nmpc_base.py' is that this file 
    is the only one to not have the angular velocity as state and instead has it defined as control input.
    Therefore, it differs fundamentally in the model and solver. 
    """
    def __init__(self, overwrite: bool = False):
        # Read parameters from configuration file in the robot's package
        self.read_params("controller", "nmpc", "mini_quadrotor", "FlightControlNMPCBodyRate.yaml")
        self.phys = phys

        # Create acados model & solver and generate c code
        super().__init__(overwrite)
    
    def create_acados_model(self) -> AcadosModel:
        # Model name
        model_name = "fix_qd_angvel_out_mdl"

        # Model states
        # No angular velocity as state!
        x = ca.SX.sym("x")
        y = ca.SX.sym("y")
        z = ca.SX.sym("z")
        vx = ca.SX.sym("vx")
        vy = ca.SX.sym("vy")
        vz = ca.SX.sym("vz")
        qw = ca.SX.sym("qw")    # Quaternions
        qx = ca.SX.sym("qx")
        qy = ca.SX.sym("qy")
        qz = ca.SX.sym("qz")
        states = ca.vertcat(x, y, z, vx, vy, vz, qw, qx, qy, qz)

        # Model parameters
        qwr = ca.SX.sym("qwr")  # Reference for quaternions
        qxr = ca.SX.sym("qxr")
        qyr = ca.SX.sym("qyr")
        qzr = ca.SX.sym("qzr")
        parameters = ca.vertcat(qwr, qxr, qyr, qzr)

        # Control inputs
        wx = ca.SX.sym("wx")
        wy = ca.SX.sym("wy")
        wz = ca.SX.sym("wz")
        f_u_b = ca.SX.sym("f_u_b")  # Combined force from all rotors in Body frame
        controls = ca.vertcat(wx, wy, wz, f_u_b)

        # Explicit dynamics
        ds = ca.vertcat(
            vx,
            vy,
            vz,
            2 * (qx * qz + qw * qy) * f_u_b,
            2 * (qy * qz - qw * qx) * f_u_b,
            (1 - 2 * qx**2 - 2 * qy**2) * f_u_b - phys.gravity,
            (-wx * qx - wy * qy - wz * qz) * 0.5,
            (wx * qw + wz * qy - wy * qz) * 0.5,
            (wy * qw - wz * qx + wx * qz) * 0.5,
            (wz * qw + wy * qx - wx * qy) * 0.5,
        )        
        f = ca.Function("f", [states, controls], [ds], ["state", "control_input"], ["ds"], {"allow_free": True})

        # Implicit dynamics
        x_dot = ca.SX.sym("x_dot", 10)
        f_impl = x_dot - f(states, controls)

        # Cost function
        # error = y - y_ref
        # NONLINEAR_LS = error^T @ Q @ error
        # qe = qr^* multiply q
        qe_x = qwr * qx - qw * qxr - qyr * qz + qy * qzr
        qe_y = qwr * qy - qw * qyr + qxr * qz - qx * qzr
        qe_z = -qxr * qy + qx * qyr + qwr * qz - qw * qzr

        state_y = ca.vertcat(
            x,
            y,
            z,
            vx,
            vy,
            vz,
            qwr,
            qe_x + qxr,
            qe_y + qyr,
            qe_z + qzr,
        )
        control_y = controls

        # Assemble acados model
        model = AcadosModel()
        model.name = model_name
        model.f_expl_expr = f(states, controls)  # CasADi expression for the explicit dynamics
        model.f_impl_expr = f_impl               # CasADi expression for the implicit dynamics
        model.x = states
        model.xdot = x_dot
        model.u = controls
        model.p = parameters
        model.cost_y_expr = ca.vertcat(state_y, control_y)  # NONLINEAR_LS
        model.cost_y_expr_e = state_y
        return model

    def create_acados_ocp_solver(self):
        # Create OCP object and set basic properties
        ocp = super().get_ocp()
        
        # Model dimensions
        nx = ocp.model.x.size()[0]; nu = ocp.model.u.size()[0]

        # Cost function
        # see https://docs.acados.org/python_interface/#acados_template.acados_ocp.AcadosOcpCost for details
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
            ]
        )

        R = np.diag([self.params["Rw"], self.params["Rw"], self.params["Rw"], self.params["Rc"]])
        
        ocp.cost.cost_type = "NONLINEAR_LS"
        ocp.cost.cost_type_e = "NONLINEAR_LS"
        ocp.cost.W = np.block([[Q, np.zeros((nx, nu))], [np.zeros((nu, nx)), R]])
        ocp.cost.W_e = Q    # Weight matrix at terminal shooting node (N).

        # Set constraints
        # - State box constraints bx
        ocp.constraints.idxbx = np.array([3, 4, 5])     # vx, vy, vz
        ocp.constraints.lbx = np.array([self.params["v_min"], self.params["v_min"], self.params["v_min"]])
        ocp.constraints.ubx = np.array([self.params["v_max"], self.params["v_max"], self.params["v_max"]])

        # - Terminal state box constraints bx_e
        ocp.constraints.idxbx_e = np.array([3, 4, 5])   # vx, vy, vz
        ocp.constraints.lbx_e = np.array([self.params["v_min"], self.params["v_min"], self.params["v_min"]])
        ocp.constraints.ubx_e = np.array([self.params["v_max"], self.params["v_max"], self.params["v_max"]])

        # - Input box constraints bu
        ocp.constraints.idxbu = np.array([0, 1, 2, 3])  # omega_x, omega_y, omega_z, collective_acceleration
        ocp.constraints.lbu = np.array(
            [self.params["w_min"], self.params["w_min"], self.params["w_min"], self.params["c_min"]]
        )
        ocp.constraints.ubu = np.array(
            [self.params["w_max"], self.params["w_max"], self.params["w_max"], self.params["c_max"]]
        )

        # Initial state and reference
        x_ref = np.zeros(nx)
        u_ref = np.zeros(nu)
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
        return AcadosOcpSolver(ocp, json_file=json_file_path, build=True)
    
    def get_reference_generator(self) -> QDNMPCReferenceGenerator:
        return self._reference_generator
    
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


if __name__ == "__main__":
    # Call controller class to generate c code
    overwrite = True
    nmpc = NMPCFixQdAngvelOut(overwrite)

    print("Successfully initialized acados OCP solver: ", nmpc.get_ocp_solver())
    print("T_samp: ", nmpc.params["T_samp"])
    print("T_horizon: ", nmpc.params["T_horizon"])
    print("T_step: ", nmpc.params["T_step"])
    print("N_steps: ", nmpc.params["N_steps"])
