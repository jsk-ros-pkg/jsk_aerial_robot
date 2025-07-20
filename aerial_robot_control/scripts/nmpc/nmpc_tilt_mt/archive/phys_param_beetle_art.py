import os
import yaml
import rospkg

# Read parameters from configuration file in the robot's package
rospack = rospkg.RosPack()

try:
    physical_param_path = os.path.join(rospack.get_path("beetle"), "config", "PhysParamBeetleArt.yaml")
except rospkg.common.ResourceNotFound:  # non-ROS environment
    # Fallback: construct absolute path from current file
    this_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.abspath(os.path.join(this_dir, "../../../../.."))
    physical_param_path = os.path.join(project_root, "robots/beetle/config/PhysParamBeetleArt.yaml")


with open(physical_param_path, "r") as f:
    physical_param_dict = yaml.load(f, Loader=yaml.FullLoader)
physical_params = physical_param_dict["physical"]

mass = physical_params["mass"]
gravity = physical_params["gravity"]
Ixx = physical_params["inertia_diag"][0]
Iyy = physical_params["inertia_diag"][1]
Izz = physical_params["inertia_diag"][2]
dr1 = physical_params["dr1"]
p1_b = physical_params["p1"]
dr2 = physical_params["dr2"]
p2_b = physical_params["p2"]
dr3 = physical_params["dr3"]
p3_b = physical_params["p3"]
dr4 = physical_params["dr4"]
p4_b = physical_params["p4"]
kq_d_kt = physical_params["kq_d_kt"]

t_servo = physical_params["t_servo"]  # Time constant of servo
t_rotor = physical_params["t_rotor"]  # Time constant of rotor

c0 = physical_params["c0"]
c1 = physical_params["c1"]
c2 = physical_params["c2"]
c3 = physical_params["c3"]
c4 = physical_params["c4"]

# fmt: off
# concatenate the parameters to make a new list
physical_param_list = [
    mass, gravity, Ixx, Iyy, Izz,
    kq_d_kt,
    dr1, p1_b[0], p1_b[1], p1_b[2],
    dr2, p2_b[0], p2_b[1], p2_b[2],
    dr3, p3_b[0], p3_b[1], p3_b[2],
    dr4, p4_b[0], p4_b[1], p4_b[2],
    t_rotor, t_servo,
]
# fmt: on
