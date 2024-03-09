'''
 Created by li-jinjie on 24-3-9.
'''
import os
import errno
import shutil
from abc import ABC, abstractmethod
from acados_template import AcadosModel, AcadosOcpSolver


class XrUrConverterBase(ABC):
    @abstractmethod
    def __init__(self):
        pass

    @abstractmethod
    def pose_point_2_xr_ur(self, target_xyz, target_qwxyz):
        pass


class NMPCBase(ABC):
    @abstractmethod
    def _set_name(self):
        model_name = "beetle_full_model"
        return model_name

    def __init__(self):
        self._ocp_model = self._create_acados_model(self._set_name())
        self._ocp_solver = self._create_acados_ocp_solver(self._ocp_model)
        self._xr_ur_converter = self._create_xr_ur_converter()

    def get_ocp_model(self):
        return self._ocp_model

    def get_ocp_solver(self):
        return self._ocp_solver

    def get_xr_ur_converter(self):
        return self._xr_ur_converter

    @abstractmethod
    def _create_acados_model(self, model_name: str) -> AcadosModel:
        pass

    @abstractmethod
    def _create_acados_ocp_solver(self, ocp_model: AcadosModel) -> AcadosOcpSolver:
        pass

    @abstractmethod
    def _create_xr_ur_converter(self) -> XrUrConverterBase:
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
