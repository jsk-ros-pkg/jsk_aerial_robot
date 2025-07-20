#!/usr/bin/env python
# -*- encoding: ascii -*-
import os, sys
import numpy as np
from acados_template import AcadosModel, AcadosOcpSolver, AcadosSim, AcadosSimSolver
import casadi as ca

from ..rh_base import RecedingHorizonBase
from ..tilt_qd import phys_param_beetle_omni as phys


class NominalImpedance(RecedingHorizonBase):
    def __init__(self):
        # Model name
        self.model_name = "nominal_impedance_mdl"

        self.tilt = False
        self.include_servo_model = False
        self.include_servo_derivative = False
        self.include_thrust_model = False
        self.include_cog_dist_model = False
        self.include_cog_dist_parameter = False
        self.include_impedance = True

        # Load robot specific parameters
        self.phys = phys

        # Read parameters from configuration file in the robot's package
        self.read_params("controller", "nmpc", "beetle_omni", "BeetleNMPCFullServoImp.yaml")

        self.acados_init_p = None

        # Create acados model & solver and generate c code
        super().__init__("nmpc")

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

        states = ca.vertcat(p, v, q, w)

        # Model parameters
        qwr = ca.SX.sym("qwr")  # Reference for quaternions
        qxr = ca.SX.sym("qxr")
        qyr = ca.SX.sym("qyr")
        qzr = ca.SX.sym("qzr")
        quaternion = ca.vertcat(qwr, qxr, qyr, qzr)

        # Control inputs
        fds_w = ca.SX.sym("fds_w", 3)
        tau_ds_b = ca.SX.sym("tau_ds_b", 3)

        controls = ca.vertcat(fds_w, tau_ds_b)

        # impedance parameters
        pM_imp_inv = ca.diag([1 / self.params["pMxy"], 1 / self.params["pMxy"], 1 / self.params["pMz"]])
        pD_imp = ca.diag([self.params["Qv_xy"], self.params["Qv_xy"], self.params["Qv_z"]])
        pK_imp = ca.diag([self.params["Qp_xy"], self.params["Qp_xy"], self.params["Qp_z"]])

        oM_imp_inv = ca.diag([1 / self.params["oMxy"], 1 / self.params["oMxy"], 1 / self.params["oMz"]])
        oD_imp = ca.diag([self.params["Qw_xy"], self.params["Qw_xy"], self.params["Qw_z"]])
        oK_imp = ca.diag([self.params["Qq_xy"], self.params["Qq_xy"], self.params["Qq_z"]])

        qe_x = qwr * qx - qw * qxr - qyr * qz + qy * qzr
        qe_y = qwr * qy - qw * qyr + qxr * qz - qx * qzr
        qe_z = -qxr * qy + qx * qyr + qwr * qz - qw * qzr

        qe_3d = ca.vertcat(qe_x, qe_y, qe_z)

        # Explicit dynamics (Time-derivative of states)
        ds = ca.vertcat(
            v,
            ca.mtimes(pM_imp_inv, (fds_w - ca.mtimes(pD_imp, v) - ca.mtimes(pK_imp, p))),
            (-wx * qx - wy * qy - wz * qz) / 2,
            (wx * qw + wz * qy - wy * qz) / 2,
            (wy * qw - wz * qx + wx * qz) / 2,
            (wz * qw + wy * qx - wx * qy) / 2,
            ca.mtimes(oM_imp_inv, (tau_ds_b - ca.mtimes(oD_imp, w) - ca.mtimes(oK_imp, qe_3d))),
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

        # state_y = ca.vertcat(p, v, qwr, qe_x + qxr, qe_y + qyr, qe_z + qzr, w)
        # state_y_e = state_y
        # control_y = ca.vertcat(fds_w, tau_ds_b)

        # Assemble acados model
        model = AcadosModel()
        model.name = self.model_name
        model.f_expl_expr = f(states, controls)  # CasADi expression for the explicit dynamics
        model.f_impl_expr = f_impl  # CasADi expression for the implicit dynamics
        model.x = states
        model.xdot = x_dot
        model.u = controls
        model.p = quaternion
        # model.cost_y_expr = ca.vertcat(state_y, control_y)  # NONLINEAR_LS
        # model.cost_y_expr_e = state_y_e

        return model

    def create_acados_ocp_solver(self) -> AcadosOcpSolver:
        # Get OCP object
        ocp = super().get_ocp()

        # # Model dimensions
        # nx = ocp.model.x.size()[0]
        # nu = ocp.model.u.size()[0]
        # n_param = ocp.model.p.size()[0]
        #
        # # Define weights
        # Q = np.diag(
        #     [
        #         0.0,
        #         0.0,
        #         0.0,
        #         0.0,
        #         0.0,
        #         0.0,
        #         0.0,
        #         0.0,
        #         0.0,
        #         0.0,
        #         0.0,
        #         0.0,
        #         0.0,
        #     ]
        # )
        # print("Q: \n", Q)
        #
        # R = np.diag(
        #     [
        #         0.0,
        #         0.0,
        #         0.0,
        #         0.0,
        #         0.0,
        #         0.0,
        #     ]
        # )
        # print("R: \n", R)
        #
        # # Cost function options
        # ocp.cost.cost_type = "NONLINEAR_LS"
        # ocp.cost.cost_type_e = "NONLINEAR_LS"
        # ocp.cost.W = np.block([[Q, np.zeros((nx, nu))], [np.zeros((nu, nx)), R]])
        # ocp.cost.W_e = Q  # Weight matrix at terminal shooting node (N).
        #
        # # Initial state and reference
        # x_ref = np.zeros(nx)
        # x_ref[6] = 1.0  # qw
        # u_ref = np.zeros(nu)
        # u_ref[0:2] = self.phys.mass * self.phys.gravity / 2  # ft1, ft2
        # self.acados_init_p = np.zeros(n_param)
        # self.acados_init_p[0] = 1.0  # qw
        #
        # ocp.constraints.x0 = x_ref
        # ocp.cost.yref = np.concatenate((x_ref, u_ref))
        # ocp.cost.yref_e = x_ref
        # ocp.parameter_values = self.acados_init_p
        #
        # # Solver options
        # ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
        # ocp.solver_options.hpipm_mode = "BALANCE"  # "BALANCE", "SPEED_ABS", "SPEED", "ROBUST". Default: "BALANCE".
        # # Start up flags:       [Seems only works for FULL_CONDENSING_QPOASES]
        # # 0: no warm start; 1: warm start; 2: hot start. Default: 0
        # # ocp.solver_options.qp_solver_warm_start = 1
        # ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
        # ocp.solver_options.integrator_type = "ERK"  # explicit Runge-Kutta integrator
        # ocp.solver_options.print_level = 0
        # ocp.solver_options.nlp_solver_type = "SQP_RTI"
        # ocp.solver_options.qp_solver_cond_N = self.params["N_steps"]
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
        ocp = self.get_ocp()
        nn = ocp.solver_options.N_horizon
        nx = ocp.dims.nx
        nu = ocp.dims.nu

        # Assemble state reference
        xr = np.zeros([nn + 1, nx])
        ur = np.zeros([nn, nu])

        return xr, ur

    def get_reference_generator(self):
        return self._reference_generator

    def _create_reference_generator(self):
        pass
        return None

    def create_acados_sim_solver(self, ts_sim: float, build: bool = True) -> AcadosSimSolver:
        ocp_model = super().get_acados_model()

        acados_sim = AcadosSim()
        acados_sim.model = ocp_model

        n_param = ocp_model.p.size()[0]
        self.acados_init_p = np.zeros(n_param)
        self.acados_init_p[0] = 1.0  # qw
        acados_sim.parameter_values = self.acados_init_p

        acados_sim.solver_options.T = ts_sim
        return AcadosSimSolver(acados_sim, json_file=ocp_model.name + "_acados_sim.json", build=build)


if __name__ == "__main__":
    print("Please run the gen_nmpc_code.py in the nmpc folder to generate the code for this controller.")
