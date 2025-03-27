import os, sys
from abc import abstractmethod
import numpy as np
from acados_template import AcadosOcpSolver

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))    # Add parent directory to path to allow relative imports
from rh_base import RecedingHorizonBase


class QDMHEBase(RecedingHorizonBase):
    """
    Base class for all quadrotor MHE estimators.
    The child classes only have specifications which define the estimator specifications.
    
    :param bool overwrite: Flag to overwrite existing c generated code for the OCP solver. Default: False
    """
    def __init__(self, overwrite: bool = False):
        super().__init__("mhe", overwrite)

    def compute_dynamical_model(self):
        pass

    @abstractmethod
    def get_weights(self):
        pass

    def create_acados_ocp_solver(self):
        # Create OCP object and set basic properties
        ocp = super().get_ocp()
        
        # Model dimensions
        nx = ocp.model.x.size()[0]; nw = ocp.model.u.size()[0]
        n_meas = ocp.model.cost_y_expr.size()[0] - nw
        
        # Get weights from parametrization child file
        Q_R, R_Q, Q_P = self.get_weights()

        # Cost function options
        ocp.cost.cost_type_0 = "NONLINEAR_LS"
        ocp.cost.cost_type = "NONLINEAR_LS"
        ocp.cost.cost_type_e = "NONLINEAR_LS"
        # Concatenate to create diagonal matrix: W_0 = diag(Q_R, R_Q, Q_P)
        W = np.block([[Q_R, np.zeros((n_meas, nw))], [np.zeros((nw, n_meas)), R_Q]])
        ocp.cost.W_0 = np.block([[W, np.zeros((n_meas + nw, nx))], [np.zeros((nx, n_meas + nw)), Q_P]])
        ocp.cost.W = W
        ocp.cost.W_e = Q_R      # Weight matrix at terminal shooting node (N)
        print("W_0: \n", ocp.cost.W_0)
        print("W: \n", ocp.cost.W)
        print("W_e: \n", ocp.cost.W_e)

        # Note: no constraints set

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
        solver = AcadosOcpSolver(ocp, json_file=json_file_path, build=True)
        print("Generated C code for acados solver successfully to " + os.getcwd())
        return solver