"""
 Created by li-jinjie on 24-11-29.
"""
import os
import errno
import shutil
import numpy as np
from tf_conversions import transformations as tf
from abc import ABC, abstractmethod
from acados_template import AcadosModel, AcadosOcpSolver, AcadosSim, AcadosSimSolver


class RecedingHorizonBase(ABC):
    def __init__(self, is_build: bool = True):
        self._ocp_model = self.create_acados_model(self.set_name())
        self._ocp_solver = self.create_acados_ocp_solver(self._ocp_model, is_build)

    def get_ocp_model(self) -> AcadosModel:
        return self._ocp_model

    def get_ocp_solver(self) -> AcadosOcpSolver:
        return self._ocp_solver

    @abstractmethod
    def set_name(self) -> str:
        return "xxx_mdl"

    @abstractmethod
    def create_acados_model(self, model_name: str) -> AcadosModel:
        pass

    @abstractmethod
    def create_acados_ocp_solver(self, ocp_model: AcadosModel, is_build: bool) -> AcadosOcpSolver:
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
