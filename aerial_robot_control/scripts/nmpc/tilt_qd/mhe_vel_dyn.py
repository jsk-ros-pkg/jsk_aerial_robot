'''
 Created by jinjie on 24/11/29.
'''

import sys
import numpy as np
import yaml
import rospkg
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
from tf_conversions import transformations as tf
import casadi as ca

from nmpc_base import NMPCBase, XrUrConverterBase

from phys_param_beetle_art import *

# read parameters from yaml
rospack = rospkg.RosPack()

mhe_param_path = os.path.join(rospack.get_path("beetle"), "config", "WrenchEstVelMHE.yaml")
with open(mhe_param_path, "r") as f:
    mhe_param_dict = yaml.load(f, Loader=yaml.FullLoader)
mhe_params = mhe_param_dict["controller"]["mhe"]
mhe_params["N_node"] = int(mhe_params["T_pred"] / mhe_params["T_integ"])


class MHEVelDyn(NMPCBase):
    def __init__(self):
        super(MHEVelDyn, self).__init__()
        self.t_servo = t_servo

    def set_name(self) -> str:
        model_name = "mhe_vel_dyn_mdl"
        return model_name

    def set_ts_ctrl(self) -> float:
        return mhe_params["T_samp"]

    def create_acados_model(self, model_name: str) -> AcadosModel:
        # model states
        v_w = ca.SX.sym("v", 3)
        omega_g = ca.SX.sym("omega", 3)
        f_d_w = ca.SX.sym("fd", 3)  # world frame
        tau_d_g = ca.SX.sym("tau_d", 3)  # cog frame

        states = ca.vertcat(v_w, omega_g, f_d_w, tau_d_g)

        # sensor
        measurements = ca.vertcat(v_w, omega_g)

        # parameters
        f_u_w = ca.SX.sym("f_u", 3)
        tau_u_g = ca.SX.sym("tau_u", 3)

        controls = ca.vertcat(f_u_w, tau_u_g)  # u

        # process noise
        w_f = ca.SX.sym("w_f", 3)
        w_tau = ca.SX.sym("w_tau", 3)

        noise = ca.vertcat(w_f, w_tau)

        # inertial
        iv = ca.diag([Ixx, Iyy, Izz])
        inv_iv = ca.diag([1 / Ixx, 1 / Iyy, 1 / Izz])
        g_i = np.array([0, 0, -gravity])

        # dynamic model
        ds = ca.vertcat(
            (f_u_w + f_d_w) / mass + g_i,
            ca.mtimes(inv_iv, (-ca.cross(omega_g, ca.mtimes(iv, omega_g)) + tau_u_g + tau_d_g)),
            w_f,
            w_tau,
        )

        # function
        func = ca.Function("func", [states, noise], [ds], ["state", "noise"], ["ds"], {"allow_free": True})

        # NONLINEAR_LS = error^T @ Q @ error; error = y - y_ref
        cost_state_y = ca.vertcat(v_w, omega_g, f_d_w, tau_d_g)  # for arriving cost
        cost_measurement_y = ca.vertcat(v_w, omega_g)
        cost_noise_y = ca.vertcat(w_f, w_tau)

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
        model.p = controls

        model.cost_y_expr_0 = ca.vertcat(cost_measurement_y, cost_noise_y, cost_state_y)  # y, u, x
        model.cost_y_expr = ca.vertcat(cost_measurement_y, cost_noise_y)  # y, u
        model.cost_y_expr_e = cost_measurement_y  # y

        return model

    def create_acados_ocp_solver(self, ocp_model: AcadosModel, is_build: bool) -> AcadosOcpSolver:
        nx = ocp_model.x.size()[0]
        nw = ocp_model.u.size()[0]
        ny = ocp_model.cost_y_expr.size()[0] - nw
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
                mhe_params["P_v"],
                mhe_params["P_v"],
                mhe_params["P_v"],
                mhe_params["P_omega"],
                mhe_params["P_omega"],
                mhe_params["P_omega"],
                mhe_params["P_f_d"],
                mhe_params["P_f_d"],
                mhe_params["P_f_d"],
                mhe_params["P_tau_d"],
                mhe_params["P_tau_d"],
                mhe_params["P_tau_d"],
            ]
        )
        print("Q_P: \n", Q_P)

        Q_R = np.diag(
            [
                mhe_params["R_v"],
                mhe_params["R_v"],
                mhe_params["R_v"],
                mhe_params["R_omega"],
                mhe_params["R_omega"],
                mhe_params["R_omega"],
            ]
        )
        print("Q_R: \n", Q_R)

        R_Q = np.diag(
            [
                mhe_params["Q_w_f"],
                mhe_params["Q_w_f"],
                mhe_params["Q_w_f"],
                mhe_params["Q_w_tau"],
                mhe_params["Q_w_tau"],
                mhe_params["Q_w_tau"],
            ]
        )
        print("R_Q: \n", R_Q)

        ocp.cost.cost_type_0 = "NONLINEAR_LS"
        # concatenate the diagonal matrix: W_0 = diag(Q_R, R_Q, Q_P)
        W = np.block([[Q_R, np.zeros((ny, nw))], [np.zeros((nw, ny)), R_Q]])
        ocp.cost.W_0 = np.block([[W, np.zeros((ny + nw, nx))], [np.zeros((nx, ny + nw)), Q_P]])

        print("W_0: \n", ocp.cost.W_0)

        ocp.cost.cost_type = "NONLINEAR_LS"
        ocp.cost.W = np.block([[Q_R, np.zeros((ny, nw))], [np.zeros((nw, ny)), R_Q]])

        ocp.cost.cost_type_e = "NONLINEAR_LS"
        ocp.cost.W_e = Q_R  # weight matrix at terminal shooting node (N).

        # # set constraints

        # initial state
        ocp.cost.yref_0 = np.zeros(ny + nw + nx)
        ocp.cost.yref = np.zeros(ny + nw)
        ocp.cost.yref_e = np.zeros(ny)

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

    def create_xr_ur_converter(self):
        return XrUrConverter()


class XrUrConverter(XrUrConverterBase):
    def __init__(self):
        super(XrUrConverter, self).__init__()

    def _set_nx_nu(self):
        self.nx = 21
        self.nu = 8

    def _set_physical_params(self):
        self.p1_b = p1_b
        self.p2_b = p2_b
        self.p3_b = p3_b
        self.p4_b = p4_b
        self.dr1 = dr1
        self.dr2 = dr2
        self.dr3 = dr3
        self.dr4 = dr4
        self.kq_d_kt = kq_d_kt

        self.mass = mass
        self.gravity = gravity

        self.alloc_mat_pinv = self._get_alloc_mat_pinv()
        self.ocp_N = mhe_params["N_node"]

    def pose_point_2_xr_ur(self, target_xyz, target_rpy):
        roll = target_rpy.item(0)
        pitch = target_rpy.item(1)
        yaw = target_rpy.item(2)

        q = tf.quaternion_from_euler(roll, pitch, yaw, axes="sxyz")
        target_qwxyz = np.array([[q[3], q[0], q[1], q[2]]]).T

        # convert [0,0,gravity] to body frame
        q_inv = tf.quaternion_inverse(q)
        rot = tf.quaternion_matrix(q_inv)
        fg_i = np.array([0, 0, self.mass * self.gravity, 0])
        fg_b = rot @ fg_i
        target_wrench = np.array([[fg_b.item(0), fg_b.item(1), fg_b.item(2), 0, 0, 0]]).T

        # a quicker method if alloc_mat is dynamic:  x, _, _, _ = np.linalg.lstsq(alloc_mat, target_wrench, rcond=None)
        x = self.alloc_mat_pinv @ target_wrench

        a1_ref = np.arctan2(x[0, 0], x[1, 0])
        ft1_ref = np.sqrt(x[0, 0] ** 2 + x[1, 0] ** 2)
        a2_ref = np.arctan2(x[2, 0], x[3, 0])
        ft2_ref = np.sqrt(x[2, 0] ** 2 + x[3, 0] ** 2)
        a3_ref = np.arctan2(x[4, 0], x[5, 0])
        ft3_ref = np.sqrt(x[4, 0] ** 2 + x[5, 0] ** 2)
        a4_ref = np.arctan2(x[6, 0], x[7, 0])
        ft4_ref = np.sqrt(x[6, 0] ** 2 + x[7, 0] ** 2)

        # get x and u, set reference
        ocp_N = self.ocp_N

        xr = np.zeros([ocp_N + 1, self.nx])
        xr[:, 0] = target_xyz.item(0)  # x
        xr[:, 1] = target_xyz.item(1)  # y
        xr[:, 2] = target_xyz.item(2)  # z
        xr[:, 6] = target_qwxyz.item(0)  # qx
        xr[:, 7] = target_qwxyz.item(1)  # qx
        xr[:, 8] = target_qwxyz.item(2)  # qy
        xr[:, 9] = target_qwxyz.item(3)  # qz
        xr[:, 13] = a1_ref
        xr[:, 14] = a2_ref
        xr[:, 15] = a3_ref
        xr[:, 16] = a4_ref
        xr[:, 17] = ft1_ref
        xr[:, 18] = ft2_ref
        xr[:, 19] = ft3_ref
        xr[:, 20] = ft4_ref

        ur = np.zeros([ocp_N, self.nu])

        return xr, ur

if __name__ == "__main__":
    nmpc = MHEVelDyn()

    acados_ocp_solver = nmpc.get_ocp_solver()
    print("Successfully initialized acados ocp: ", acados_ocp_solver.acados_ocp)
    print("number of states: ", acados_ocp_solver.acados_ocp.dims.nx)
    print("number of controls: ", acados_ocp_solver.acados_ocp.dims.nu)
    print("number of parameters: ", acados_ocp_solver.acados_ocp.dims.np)
    print("T_samp: ", mhe_params["T_samp"])
    print("T_pred: ", mhe_params["T_pred"])
    print("T_integ: ", mhe_params["T_integ"])
    print("N_node: ", mhe_params["N_node"])
