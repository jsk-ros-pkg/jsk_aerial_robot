"""
 Created by li-jinjie on 24-3-9.
"""
import os
import errno
import shutil
import numpy as np
from tf_conversions import transformations as tf
from abc import ABC, abstractmethod
from acados_template import AcadosModel, AcadosOcpSolver, AcadosSim, AcadosSimSolver


class XrUrConverterBase(ABC):
    @abstractmethod
    def __init__(self):
        self._set_nx_nu()
        self._set_physical_params()

    @abstractmethod
    def _set_nx_nu(self):
        self.nx = int()
        self.nu = int()

    @abstractmethod
    def _set_physical_params(self):
        self.p1_b = np.array([float(), float(), float()])
        self.p2_b = np.array([float(), float(), float()])
        self.p3_b = np.array([float(), float(), float()])
        self.p4_b = np.array([float(), float(), float()])
        self.dr1 = float()
        self.dr2 = float()
        self.dr3 = float()
        self.dr4 = float()
        self.kq_d_kt = float()

        self.mass = float()
        self.gravity = float()

        self.alloc_mat = self._get_alloc_mat()
        self.alloc_mat_pinv = self._get_alloc_mat_pinv()
        self.ocp_N = int()

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

        ur = np.zeros([ocp_N, self.nu])
        ur[:, 0] = ft1_ref
        ur[:, 1] = ft2_ref
        ur[:, 2] = ft3_ref
        ur[:, 3] = ft4_ref

        return xr, ur

    def _get_alloc_mat(self):
        p1_b, p2_b, p3_b, p4_b = self.p1_b, self.p2_b, self.p3_b, self.p4_b
        dr1, dr2, dr3, dr4 = self.dr1, self.dr2, self.dr3, self.dr4
        kq_d_kt = self.kq_d_kt

        # get allocation matrix
        alloc_mat = np.zeros((6, 8))
        sqrt_p1b_xy = np.sqrt(self.p1_b[0] ** 2 + self.p1_b[1] ** 2)
        sqrt_p2b_xy = np.sqrt(self.p2_b[0] ** 2 + self.p2_b[1] ** 2)
        sqrt_p3b_xy = np.sqrt(self.p3_b[0] ** 2 + self.p3_b[1] ** 2)
        sqrt_p4b_xy = np.sqrt(self.p4_b[0] ** 2 + self.p4_b[1] ** 2)

        # - force
        alloc_mat[0, 0] = p1_b[1] / sqrt_p1b_xy
        alloc_mat[1, 0] = -p1_b[0] / sqrt_p1b_xy
        alloc_mat[2, 1] = 1

        alloc_mat[0, 2] = p2_b[1] / sqrt_p2b_xy
        alloc_mat[1, 2] = -p2_b[0] / sqrt_p2b_xy
        alloc_mat[2, 3] = 1

        alloc_mat[0, 4] = p3_b[1] / sqrt_p3b_xy
        alloc_mat[1, 4] = -p3_b[0] / sqrt_p3b_xy
        alloc_mat[2, 5] = 1

        alloc_mat[0, 6] = p4_b[1] / sqrt_p4b_xy
        alloc_mat[1, 6] = -p4_b[0] / sqrt_p4b_xy
        alloc_mat[2, 7] = 1

        # - torque
        alloc_mat[3, 0] = -dr1 * kq_d_kt * p1_b[1] / sqrt_p1b_xy + p1_b[0] * p1_b[2] / sqrt_p1b_xy
        alloc_mat[4, 0] = dr1 * kq_d_kt * p1_b[0] / sqrt_p1b_xy + p1_b[1] * p1_b[2] / sqrt_p1b_xy
        alloc_mat[5, 0] = -p1_b[0] ** 2 / sqrt_p1b_xy - p1_b[1] ** 2 / sqrt_p1b_xy

        alloc_mat[3, 1] = p1_b[1]
        alloc_mat[4, 1] = -p1_b[0]
        alloc_mat[5, 1] = -dr1 * kq_d_kt

        alloc_mat[3, 2] = -dr2 * kq_d_kt * p2_b[1] / sqrt_p2b_xy + p2_b[0] * p2_b[2] / sqrt_p2b_xy
        alloc_mat[4, 2] = dr2 * kq_d_kt * p2_b[0] / sqrt_p2b_xy + p2_b[1] * p2_b[2] / sqrt_p2b_xy
        alloc_mat[5, 2] = -p2_b[0] ** 2 / sqrt_p2b_xy - p2_b[1] ** 2 / sqrt_p2b_xy

        alloc_mat[3, 3] = p2_b[1]
        alloc_mat[4, 3] = -p2_b[0]
        alloc_mat[5, 3] = -dr2 * kq_d_kt

        alloc_mat[3, 4] = -dr3 * kq_d_kt * p3_b[1] / sqrt_p3b_xy + p3_b[0] * p3_b[2] / sqrt_p3b_xy
        alloc_mat[4, 4] = dr3 * kq_d_kt * p3_b[0] / sqrt_p3b_xy + p3_b[1] * p3_b[2] / sqrt_p3b_xy
        alloc_mat[5, 4] = -p3_b[0] ** 2 / sqrt_p3b_xy - p3_b[1] ** 2 / sqrt_p3b_xy

        alloc_mat[3, 5] = p3_b[1]
        alloc_mat[4, 5] = -p3_b[0]
        alloc_mat[5, 5] = -dr3 * kq_d_kt

        alloc_mat[3, 6] = -dr4 * kq_d_kt * p4_b[1] / sqrt_p4b_xy + p4_b[0] * p4_b[2] / sqrt_p4b_xy
        alloc_mat[4, 6] = dr4 * kq_d_kt * p4_b[0] / sqrt_p4b_xy + p4_b[1] * p4_b[2] / sqrt_p4b_xy
        alloc_mat[5, 6] = -p4_b[0] ** 2 / sqrt_p4b_xy - p4_b[1] ** 2 / sqrt_p4b_xy

        alloc_mat[3, 7] = p4_b[1]
        alloc_mat[4, 7] = -p4_b[0]
        alloc_mat[5, 7] = -dr4 * kq_d_kt

        return alloc_mat

    def _get_alloc_mat_pinv(self):
        alloc_mat = self._get_alloc_mat()
        alloc_mat_pinv = np.linalg.pinv(alloc_mat)
        return alloc_mat_pinv


class NMPCBase(ABC):
    def __init__(self, is_build: bool = True):
        self.ts_ctrl = self.set_ts_ctrl()

        self._ocp_model = self.create_acados_model(self.set_name())
        self._ocp_solver = self.create_acados_ocp_solver(self._ocp_model, is_build)
        self._xr_ur_converter = self.create_xr_ur_converter()

    def get_ocp_model(self) -> AcadosModel:
        return self._ocp_model

    def get_ocp_solver(self) -> AcadosOcpSolver:
        return self._ocp_solver

    def get_xr_ur_converter(self) -> XrUrConverterBase:
        return self._xr_ur_converter

    @abstractmethod
    def set_name(self) -> str:
        return "xxx_mdl"

    @abstractmethod
    def set_ts_ctrl(self) -> float:
        return float()

    @abstractmethod
    def create_acados_model(self, model_name: str) -> AcadosModel:
        pass

    @staticmethod
    def create_acados_sim_solver(ocp_model: AcadosModel, ts_sim: float, is_build: bool) -> AcadosSimSolver:
        acados_sim = AcadosSim()
        acados_sim.model = ocp_model

        n_params = ocp_model.p.size()[0]
        acados_sim.dims.np = n_params
        acados_sim.parameter_values = np.zeros(n_params)

        acados_sim.solver_options.T = ts_sim
        acados_sim_solver = AcadosSimSolver(acados_sim, json_file=ocp_model.name + "_acados_sim.json", build=is_build)
        return acados_sim_solver

    @abstractmethod
    def create_acados_ocp_solver(self, ocp_model: AcadosModel, is_build: bool) -> AcadosOcpSolver:
        pass

    @abstractmethod
    def create_xr_ur_converter(self) -> XrUrConverterBase:
        pass

    @staticmethod
    def _mkdir(directory, overwrite=False):
        safe_mkdir_recursive(directory, overwrite)


def safe_mkdir_recursive(directory, overwrite=False):
    if not os.path.exists(directory):
        try:
            os.makedirs(directory)
        except OSError as exc:
            if exc.errno == errno.EEXIST and os.path.isdir(directory):
                pass
            else:
                raise
    else:
        if overwrite:
            try:
                shutil.rmtree(directory)
            except:
                print("Error while removing directory {}".format(directory))
