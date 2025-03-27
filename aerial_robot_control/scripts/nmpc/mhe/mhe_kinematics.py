import os, sys
import numpy as np
from acados_template import AcadosModel, AcadosOcpSolver
import casadi as ca
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))    # Add parent directory to path to allow relative imports
from rh_base import RecedingHorizonBase

from phys_param_beetle_omni import *        # Define physical parameters


class MHEKinematics(RecedingHorizonBase):
    """
    General kinematics-based Moving Horizon Estimation (MHE) to implement state estimation for any robot.

    :opt param bool overwrite: Flag to overwrite existing c generated code for the OCP solver. Default: False
    """
    def __init__(self, overwrite: bool = False):
        # Read parameters from configuration file in the robot's package
        self.read_params("estimation", "mhe", "beetle_omni", "StateEstimationMHE.yaml")
        
        # Create acados model & solver and generate c code
        super().__init__("wrench_est", overwrite)

    def create_acados_model(self) -> AcadosModel:
        # Model name
        model_name = "mhe_kinematics_mdl"
        
        # Model states
        p = ca.SX.sym("p", 3)           # World frame
        v = ca.SX.sym("v", 3)           # World frame
        a_sf = ca.SX.sym("a_sf", 3)     # Body frame
        qw = ca.SX.sym("qw")            # Quaternions
        qx = ca.SX.sym("qx")
        qy = ca.SX.sym("qy")
        qz = ca.SX.sym("qz")
        q = ca.vertcat(qw, qx, qy, qz)
        wx = ca.SX.sym("wx")
        wy = ca.SX.sym("wy")
        wz = ca.SX.sym("wz")
        w = ca.vertcat(wx, wy, wz)
        states = ca.vertcat(p, v, a_sf, q, w)

        # Model parameters
        qwr = ca.SX.sym("qwr")          # Reference for quaternions
        qxr = ca.SX.sym("qxr")
        qyr = ca.SX.sym("qyr")
        qzr = ca.SX.sym("qzr")
        parameters = ca.vertcat(qwr, qxr, qyr, qzr)

        # Model noise
        w_a_sf = ca.SX.sym("w_a_sf", 3)
        w_w = ca.SX.sym("w_w", 3)
        noise = ca.vertcat(w_a_sf, w_w)

        # Transformation matrix
        row_1 = ca.horzcat(
            ca.SX(1 - 2 * qy ** 2 - 2 * qz ** 2), ca.SX(2 * qx * qy - 2 * qw * qz), ca.SX(2 * qx * qz + 2 * qw * qy)
        )
        row_2 = ca.horzcat(
            ca.SX(2 * qx * qy + 2 * qw * qz), ca.SX(1 - 2 * qx ** 2 - 2 * qz ** 2), ca.SX(2 * qy * qz - 2 * qw * qx)
        )
        row_3 = ca.horzcat(
            ca.SX(2 * qx * qz - 2 * qw * qy), ca.SX(2 * qy * qz + 2 * qw * qx), ca.SX(1 - 2 * qx ** 2 - 2 * qy ** 2)
        )
        rot_wb = ca.vertcat(row_1, row_2, row_3)

        # Gravity
        g_w = np.array([0, 0, -gravity])

        # Explicit dynamics
        ds = ca.vertcat(
            v,
            ca.mtimes(rot_wb, a_sf) + g_w,
            w_a_sf,
            (-wx * qx - wy * qy - wz * qz) / 2,
            ( wx * qw + wz * qy - wy * qz) / 2,
            ( wy * qw - wz * qx + wx * qz) / 2,
            ( wz * qw + wy * qx - wx * qy) / 2,
            w_w
        )
        f = ca.Function("func", [states, noise], [ds], ["state", "noise"], ["ds"], {"allow_free": True})

        # Implicit dynamics
        x_dot = ca.SX.sym("x_dot", states.size()[0])
        f_impl = x_dot - f(states, noise)

        # Cost function
        # error = y - y_ref
        # NONLINEAR_LS = error^T @ Q @ error
        # qe = qr^* multiply q
        qe_x =  qwr * qx - qw * qxr - qyr * qz + qy * qzr
        qe_y =  qwr * qy - qw * qyr + qxr * qz - qx * qzr
        qe_z = -qxr * qy + qx * qyr + qwr * qz - qw * qzr

        # Sensor function
        meas_y = ca.vertcat(p, a_sf, qwr, qe_x + qxr, qe_y + qyr, qe_z + qzr, w)
        meas_y_e = meas_y

        # Assemble acados model
        model = AcadosModel()
        model.name = model_name
        model.f_expl_expr = f(states, noise)  # CasADi expression for the explicit dynamics
        model.f_impl_expr = f_impl            # CasADi expression for the implicit dynamics
        model.x = states
        model.xdot = x_dot
        model.u = noise
        model.p = parameters
        # TODO: the error for quaternion in states should also be considered as quaternion error.
        model.cost_y_expr_0 = ca.vertcat(meas_y, noise, states)  # y, u, x
        model.cost_y_expr = ca.vertcat(meas_y, noise)            # y, u
        model.cost_y_expr_e = meas_y_e                           # y
        return model

    def create_acados_ocp_solver(self) -> AcadosOcpSolver:
        # Create OCP object and set basic properties
        ocp = super().get_ocp()
        
        # Model dimensions
        nx = ocp.model.x.size()[0]; nw = ocp.model.u.size()[0]
        n_meas = ocp.model.cost_y_expr.size()[0] - nw

        # Cost function
        # see https://docs.acados.org/python_interface/#acados_template.acados_ocp.AcadosOcpCost for details
        Q_P = np.diag(
            [
                self.params["P_p"],
                self.params["P_p"],
                self.params["P_p"],
                self.params["P_v"],
                self.params["P_v"],
                self.params["P_v"],
                self.params["P_a_sf"],
                self.params["P_a_sf"],
                self.params["P_a_sf"],
                0,
                self.params["P_q"],
                self.params["P_q"],
                self.params["P_q"],
                self.params["P_omega"],
                self.params["P_omega"],
                self.params["P_omega"],
            ]
        )

        Q_R = np.diag(
            [
                1 / (self.params["R_p"] ** 2),
                1 / (self.params["R_p"] ** 2),
                1 / (self.params["R_p"] ** 2),
                1 / (self.params["R_a_sf"] ** 2),
                1 / (self.params["R_a_sf"] ** 2),
                1 / (self.params["R_a_sf"] ** 2),
                1 / (self.params["R_q"] ** 2),      # TODO: This error should also be considered as quaternion error.
                1 / (self.params["R_q"] ** 2),
                1 / (self.params["R_q"] ** 2),
                1 / (self.params["R_q"] ** 2),
                1 / (self.params["R_omega"] ** 2),
                1 / (self.params["R_omega"] ** 2),
                1 / (self.params["R_omega"] ** 2),
            ]
        )

        R_Q = np.diag(
            [
                self.params["Q_w_a_sf"],
                self.params["Q_w_a_sf"],
                self.params["Q_w_a_sf"],
                self.params["Q_w_omega"],
                self.params["Q_w_omega"],
                self.params["Q_w_omega"],
            ]
        )

        # Cost function options
        ocp.cost.cost_type_0 = "NONLINEAR_LS"
        ocp.cost.cost_type = "NONLINEAR_LS"
        ocp.cost.cost_type_e = "NONLINEAR_LS"
        # Concatenate the diagonal matrix: W_0 = diag(Q_R, R_Q, Q_P)
        W = np.block([[Q_R, np.zeros((n_meas, nw))], [np.zeros((nw, n_meas)), R_Q]])
        ocp.cost.W_0 = np.block([[W, np.zeros((n_meas + nw, nx))], [np.zeros((nx, n_meas + nw)), Q_P]])
        ocp.cost.W = np.block([[Q_R, np.zeros((n_meas, nw))], [np.zeros((nw, n_meas)), R_Q]])
        ocp.cost.W_e = Q_R      # Weight matrix at terminal shooting node (N).

        # Reference
        ocp.cost.yref_0 = np.zeros(n_meas + nw + nx)
        ocp.cost.yref = np.zeros(n_meas + nw)
        ocp.cost.yref_e = np.zeros(n_meas)

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


if __name__ == "__main__":
    # Call MHE class to generate c code
    overwrite = True
    mhe = MHEKinematics(overwrite)

    acados_ocp_solver = mhe.get_ocp_solver()
    print("Successfully initialized acados OCP solver: ", acados_ocp_solver.acados_ocp)
    print("Number of states: ", acados_ocp_solver.acados_ocp.dims.nx)
    print("Number of noises: ", acados_ocp_solver.acados_ocp.dims.nu)
    print("Number of parameters: ", acados_ocp_solver.acados_ocp.dims.np)
    print("T_samp: ", mhe.params["T_samp"])
    print("T_horizon: ", mhe.params["T_horizon"])
    print("T_step: ", mhe.params["T_step"])
    print("N_steps: ", mhe.params["N_steps"])
