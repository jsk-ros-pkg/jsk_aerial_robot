"""
 Created by li-jinjie on 24-10-17.
"""

import os
import yaml
import rospkg

# read parameters from yaml
rospack = rospkg.RosPack()
param_path = os.path.join(rospack.get_path("gimbalrotor"), "config", "PhysParamBirotor.yaml")
with open(param_path, "r") as f:
    param_dict = yaml.load(f, Loader=yaml.FullLoader)

physical_params = param_dict["physical"]
mass = physical_params["mass"]
gravity = physical_params["gravity"]
Ixx = physical_params["inertia_diag"][0]
Iyy = physical_params["inertia_diag"][1]
Izz = physical_params["inertia_diag"][2]
dr1 = physical_params["dr1"]
p1_b = physical_params["p1"]
dr2 = physical_params["dr2"]
p2_b = physical_params["p2"]
kq_d_kt = physical_params["kq_d_kt"]

# servo parameters
## 1-ord
t_servo = physical_params["t_servo"]  # time constant of servo

## 2-ord
kps = physical_params["kps"]
kds = physical_params["kds"]
mus = physical_params["mus"]

i_sxx = physical_params["servo_inertia_x"]
