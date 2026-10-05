# 2025-10-08 copy parameters to be independent of any ROS package
# these parameters are used in GitHub action to ensure the correctness of the nmpc code
# In real usage, these parameters will be reloaded from URDF.

mass = 1.0564  # kg
gravity = 9.798  # m/s^2
Ixx = 0.00867032  # kg m^2
Iyy = 0.00867103
Izz = 0.00979117
dr1 = -1
dr2 = 1
dr3 = -1
dr4 = 1
p1_b = [-0.0865752, -0.085, 0.0108557]
p2_b = [0.0834248, -0.085, 0.0108557]
p3_b = [0.0834248, 0.085, 0.0108557]
p4_b = [-0.0865752, 0.085, 0.0108557]
kq_d_kt = 0.011

# concatenate the parameters to make a new list
# fmt: off
physical_param_list = [
    mass, gravity, Ixx, Iyy, Izz,
    kq_d_kt,
    dr1, p1_b[0], p1_b[1], p1_b[2],
    dr2, p2_b[0], p2_b[1], p2_b[2],
    dr3, p3_b[0], p3_b[1], p3_b[2],
    dr4, p4_b[0], p4_b[1], p4_b[2],
    0.0, 0.0,
]
# fmt: on

physical_param_list.extend([0.0, 0.0, 0.0])
physical_param_list.extend([1.0, 0.0, 0.0, 0.0])  # to compatible with end-effectors.
