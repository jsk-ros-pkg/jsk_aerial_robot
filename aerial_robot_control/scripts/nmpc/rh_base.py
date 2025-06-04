import os, sys
import errno
import shutil
import numpy as np
from abc import ABC, abstractmethod
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
import yaml
import rospkg


class RecedingHorizonBase(ABC):
    """
    Base class for the receding horizon methods NMPC and MHE.
    
    :param string method: Sets correct path to save c generated code and is either "nmpc" or "wrench_est".
    :opt param bool overwrite: Flag to overwrite existing c generated code for the OCP solver. Default: False
    """
    def __init__(self, method, overwrite: bool = False):
        self._acados_model = self.create_acados_model()
        self._create_acados_ocp()
        self._mkdir(method, self._acados_model.name, overwrite)
        self._ocp_solver = self.create_acados_ocp_solver()  # Goes into child class's implementation

    def read_params(self, mode, method, robot_package, file_name):
        # Read parameters from configuration file in the robot's package
        # 'mode' is either "control" or "estimation"
        # 'method' is either "nmpc" or "mhe"
        rospack = rospkg.RosPack()
        param_path = os.path.join(rospack.get_path(robot_package), "config", file_name)
        try:
            with open(param_path, "r") as f:
                param_dict = yaml.load(f, Loader=yaml.FullLoader)
            self.params = param_dict[mode][method]
        except FileNotFoundError:
            raise FileNotFoundError(f"Configuration file {param_path} not found.")
        except KeyError:
            raise KeyError(f"Mode or method not found in configuration file {param_path}.")
        
        # Compute number of shooting intervals or "steps" (or "nodes") along the horizon
        self.params["N_steps"] = int(self.params["T_horizon"] / self.params["T_step"])
    
    def _create_acados_ocp(self) -> AcadosOcp:
        # Gets called from child class's implementation
        # Create OCP object and set basic properties
        self._ocp = AcadosOcp()
        acados_source_path = os.environ["ACADOS_SOURCE_DIR"]        # Get acados source path from environment variable
        self._ocp.acados_include_path = acados_source_path + "/include"
        self._ocp.acados_lib_path = acados_source_path + "/lib"
        self._ocp.model = self._acados_model

        # TODO I think not needed but not sure
        sys.path.insert(0, acados_source_path)

        # Set parameters
        self._ocp.dims.N = self.params["N_steps"]                   # Number of time steps along the prediction horizon
        self._ocp.dims.np = self._acados_model.p.size()[0]             # Number of parameters
        self._ocp.parameter_values = np.zeros(self._ocp.dims.np)    # Initialize parameters with zeros
    
    def get_acados_model(self) -> AcadosModel:
        return self._acados_model
    
    def get_ocp(self) -> AcadosOcp:
        return self._ocp

    def get_ocp_solver(self) -> AcadosOcpSolver:
        return self._ocp_solver

    @abstractmethod
    def create_acados_model(self) -> AcadosModel:
        pass
    
    @abstractmethod
    def create_acados_ocp_solver(self) -> AcadosOcpSolver:
        pass

    @staticmethod
    def _mkdir(method, model_name, overwrite: bool = False):
        # Make a directory for generating cpp files of the acados model and solver
        # 'method' is either "nmpc" or "wrench_est"
        rospack = rospkg.RosPack()
        folder_path = os.path.join(rospack.get_path("aerial_robot_control"), "include", "aerial_robot_control", method,
                                   model_name)
        safe_mkdir_recursive(folder_path, overwrite)
        os.chdir(folder_path)   # Change working directory to the model folder (also affects inheritted classes)

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
                try:
                    os.makedirs(directory)
                except OSError as exc:
                    if exc.errno == errno.EEXIST and os.path.isdir(directory):
                        pass
                    else:
                        raise
            except:
                print("Error while removing directory {}".format(directory))
