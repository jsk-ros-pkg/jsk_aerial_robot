'''
 Created by li-jinjie on 24-12-7.
'''

import sys
import numpy as np
import yaml
import rospkg
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
import casadi as c

from rh_base import RecedingHorizonBase

from phys_param_beetle_omni import *

# read parameters from yaml
rospack = rospkg.RosPack()

mhe_param_path = os.path.join(rospack.get_path("beetle"), "config", "StateEstimationMHE.yaml")
with open(mhe_param_path, "r") as f:
    mhe_param_dict = yaml.load(f, Loader=yaml.FullLoader)
mhe_params = mhe_param_dict["controller"]["mhe"]
mhe_params["N_node"] = int(mhe_params["T_pred"] / mhe_params["T_integ"])
