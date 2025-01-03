'''
 Created by li-jinjie on 24-12-7.
'''

import sys
import numpy as np
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
import casadi as ca

from rh_base import RecedingHorizonBase

from phys_param_beetle_omni import *

# read parameters from yaml
rospack = rospkg.RosPack()

mhe_param_path = os.path.join(rospack.get_path("beetle"), "config", "StateEstimationMHE.yaml")
with open(mhe_param_path, "r") as f:
    mhe_param_dict = yaml.load(f, Loader=yaml.FullLoader)
mhe_params = mhe_param_dict["estimation"]["mhe"]
mhe_params["N_node"] = int(mhe_params["T_pred"] / mhe_params["T_integ"])


class MHEKinematics(RecedingHorizonBase):
    def __init__(self):
        super(MHEKinematics, self).__init__()
        self.t_servo = t_servo

    def set_name(self) -> str:
        model_name = "mhe_kinematics_mdl"
        return model_name

    def create_acados_model(self, model_name: str) -> AcadosModel:
        # model states
        p = ca.SX.sym("p", 3)  # world frame
        v = ca.SX.sym("v", 3)  # world frame
        a_sf = ca.SX.sym("a_sf", 3)  # body frame

        qw = ca.SX.sym("qw")
        qx = ca.SX.sym("qx")
        qy = ca.SX.sym("qy")
        qz = ca.SX.sym("qz")
        q = ca.vertcat(qw, qx, qy, qz)

        wx = ca.SX.sym("wx")
        wy = ca.SX.sym("wy")
        wz = ca.SX.sym("wz")
        w = ca.vertcat(wx, wy, wz)

        states = ca.vertcat(p, v, a_sf, q, w)

        # parameters
        qwr = ca.SX.sym("qwr")  # reference for quaternions
        qxr = ca.SX.sym("qxr")
        qyr = ca.SX.sym("qyr")
        qzr = ca.SX.sym("qzr")
        parameters = ca.vertcat(qwr, qxr, qyr, qzr)

        # process noise
        w_a_sf = ca.SX.sym("w_a_sf", 3)
        w_w = ca.SX.sym("w_w", 3)
        noise = ca.vertcat(w_a_sf, w_w)

        # gravity
        g_i = np.array([0, 0, -gravity])

        # transformation matrix
        row_1 = ca.horzcat(
            ca.SX(1 - 2 * qy ** 2 - 2 * qz ** 2), ca.SX(2 * qx * qy - 2 * qw * qz), ca.SX(2 * qx * qz + 2 * qw * qy)
        )
        row_2 = ca.horzcat(
            ca.SX(2 * qx * qy + 2 * qw * qz), ca.SX(1 - 2 * qx ** 2 - 2 * qz ** 2), ca.SX(2 * qy * qz - 2 * qw * qx)
        )
        row_3 = ca.horzcat(
            ca.SX(2 * qx * qz - 2 * qw * qy), ca.SX(2 * qy * qz + 2 * qw * qx), ca.SX(1 - 2 * qx ** 2 - 2 * qy ** 2)
        )
        rot_ib = ca.vertcat(row_1, row_2, row_3)

        # dynamic model
        ds = ca.vertcat(
            v,
            ca.mtimes(rot_ib, a_sf) + g_i,
            w_a_sf,
            (-wx * qx - wy * qy - wz * qz) / 2,
            (wx * qw + wz * qy - wy * qz) / 2,
            (wy * qw - wz * qx + wx * qz) / 2,
            (wz * qw + wy * qx - wx * qy) / 2,
            w_w
        )

        # function
        func = ca.Function("func", [states, noise], [ds], ["state", "noise"], ["ds"], {"allow_free": True})

        # NONLINEAR_LS = error^T @ Q @ error; error = y - y_ref
        # qe = qr^* multiply q
        qe_x = qwr * qx - qw * qxr - qyr * qz + qy * qzr
        qe_y = qwr * qy - qw * qyr + qxr * qz - qx * qzr
        qe_z = -qxr * qy + qx * qyr + qwr * qz - qw * qzr

        # sensor function
        meas_y = ca.vertcat(p, a_sf, qwr, qe_x + qxr, qe_y + qyr, qe_z + qzr, w)

        # acados model
        x_dot = ca.SX.sym("x_dot", states.size()[0])
        f_impl = x_dot - func(states, noise)

        model = AcadosModel()
        model.name = model_name
        model.f_expl_expr = func(states, noise)  # CasADi expression for the explicit dynamics
        model.f_impl_expr = f_impl  # CasADi expression for the implicit dynamics
        model.x = states
        model.xdot = x_dot
        model.u = noise
        model.p = parameters

        # TODO: the error for quaternion in states should also be considered as quaternion error.
        model.cost_y_expr_0 = ca.vertcat(meas_y, noise, states)  # y, u, x
        model.cost_y_expr = ca.vertcat(meas_y, noise)  # y, u
        model.cost_y_expr_e = meas_y  # y

        return model

    def create_acados_ocp_solver(self, ocp_model: AcadosModel, is_build: bool) -> AcadosOcpSolver:
        nx = ocp_model.x.size()[0]
        nw = ocp_model.u.size()[0]
        n_meas = ocp_model.cost_y_expr.size()[0] - nw
        n_params = ocp_model.p.size()[0]

        # get file path for acados
        rospack = rospkg.RosPack()
        folder_path = os.path.join(
            rospack.get_path("aerial_robot_control"), "include", "aerial_robot_control", "wrench_est", ocp_model.name
        )
        self._mkdir(folder_path)
        os.chdir(folder_path)
        # acados_models_dir = "acados_models"
        # safe_mkdir_recursive(os.path.join(os.getcwd(), acados_models_dir))

        acados_source_path = os.environ["ACADOS_SOURCE_DIR"]
        sys.path.insert(0, acados_source_path)

        # create OCP
        ocp = AcadosOcp()
        ocp.acados_include_path = acados_source_path + "/include"
        ocp.acados_lib_path = acados_source_path + "/lib"
        ocp.model = ocp_model
        ocp.dims.N = mhe_params["N_node"]

        # initialize parameters
        ocp.dims.np = n_params
        ocp.parameter_values = np.zeros(n_params)

        # cost function
        Q_P = np.diag(
            [
                mhe_params["P_p"],
                mhe_params["P_p"],
                mhe_params["P_p"],
                mhe_params["P_v"],
                mhe_params["P_v"],
                mhe_params["P_v"],
                mhe_params["P_a_sf"],
                mhe_params["P_a_sf"],
                mhe_params["P_a_sf"],
                0,
                mhe_params["P_q"],
                mhe_params["P_q"],
                mhe_params["P_q"],
                mhe_params["P_omega"],
                mhe_params["P_omega"],
                mhe_params["P_omega"],
            ]
        )
        print("Q_P: \n", Q_P)

        Q_R = np.diag(
            [
                1 / (mhe_params["R_p"] ** 2),
                1 / (mhe_params["R_p"] ** 2),
                1 / (mhe_params["R_p"] ** 2),
                1 / (mhe_params["R_a_sf"] ** 2),
                1 / (mhe_params["R_a_sf"] ** 2),
                1 / (mhe_params["R_a_sf"] ** 2),
                1 / (mhe_params["R_q"] ** 2),  # TODO: This error should also be considered as quaternion error.
                1 / (mhe_params["R_q"] ** 2),
                1 / (mhe_params["R_q"] ** 2),
                1 / (mhe_params["R_q"] ** 2),
                1 / (mhe_params["R_omega"] ** 2),
                1 / (mhe_params["R_omega"] ** 2),
                1 / (mhe_params["R_omega"] ** 2),
            ]
        )
        print("Q_R: \n", Q_R)

        R_Q = np.diag(
            [
                mhe_params["Q_w_a_sf"],
                mhe_params["Q_w_a_sf"],
                mhe_params["Q_w_a_sf"],
                mhe_params["Q_w_omega"],
                mhe_params["Q_w_omega"],
                mhe_params["Q_w_omega"],
            ]
        )
        print("R_Q: \n", R_Q)

        ocp.cost.cost_type_0 = "NONLINEAR_LS"
        # concatenate the diagonal matrix: W_0 = diag(Q_R, R_Q, Q_P)
        W = np.block([[Q_R, np.zeros((n_meas, nw))], [np.zeros((nw, n_meas)), R_Q]])
        ocp.cost.W_0 = np.block([[W, np.zeros((n_meas + nw, nx))], [np.zeros((nx, n_meas + nw)), Q_P]])

        print("W_0: \n", ocp.cost.W_0)

        ocp.cost.cost_type = "NONLINEAR_LS"
        ocp.cost.W = np.block([[Q_R, np.zeros((n_meas, nw))], [np.zeros((nw, n_meas)), R_Q]])

        ocp.cost.cost_type_e = "NONLINEAR_LS"
        ocp.cost.W_e = Q_R  # weight matrix at terminal shooting node (N).

        # # set constraints

        # initial state
        ocp.cost.yref_0 = np.zeros(n_meas + nw + nx)
        ocp.cost.yref = np.zeros(n_meas + nw)
        ocp.cost.yref_e = np.zeros(n_meas)

        # solver options
        ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
        ocp.solver_options.hpipm_mode = "BALANCE"  # "BALANCE", "SPEED_ABS", "SPEED", "ROBUST". Default: "BALANCE".
        # # 0: no warm start; 1: warm start; 2: hot start. Default: 0   Seems only works for FULL_CONDENSING_QPOASES
        # ocp.solver_options.qp_solver_warm_start = 1
        ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
        ocp.solver_options.integrator_type = "ERK"  # explicit Runge-Kutta integrator
        ocp.solver_options.print_level = 0
        ocp.solver_options.nlp_solver_type = "SQP_RTI"
        ocp.solver_options.qp_solver_cond_N = mhe_params["N_node"]
        ocp.solver_options.tf = mhe_params["T_pred"]

        # compile acados ocp
        json_file_path = os.path.join("./" + ocp_model.name + "_acados_ocp.json")
        solver = AcadosOcpSolver(ocp, json_file=json_file_path, build=is_build)

        return solver


if __name__ == "__main__":
    nmpc = MHEKinematics()

    acados_ocp_solver = nmpc.get_ocp_solver()
    print("Successfully initialized acados ocp: ", acados_ocp_solver.acados_ocp)
    print("number of states: ", acados_ocp_solver.acados_ocp.dims.nx)
    print("number of noise: ", acados_ocp_solver.acados_ocp.dims.nu)
    print("number of parameters: ", acados_ocp_solver.acados_ocp.dims.np)
    print("T_samp: ", mhe_params["T_samp"])
    print("T_pred: ", mhe_params["T_pred"])
    print("T_integ: ", mhe_params["T_integ"])
    print("N_node: ", mhe_params["N_node"])
