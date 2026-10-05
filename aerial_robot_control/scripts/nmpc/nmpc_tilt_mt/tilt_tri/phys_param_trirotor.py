# 2025-10-08 copy parameters to be independent of any ROS package
# these parameters are used in GitHub action to ensure the correctness of the nmpc code
# In real usage, these parameters will be reloaded from URDF.

mass = 1.52127  # sim: 1.52127 # kg
gravity = 9.798
Ixx = 0.025102
Iyy = 0.0190842
Izz = 0.0387961
dr1 = 1
p1_b = [0.0864353, -0.23752, 0.0259075]
dr2 = -1
p2_b = [0.0864353, 0.23748, 0.0259075]
dr3 = 1
p3_b = [-0.211065, -0.00002, 0.0259075]
kq_d_kt = 0.0172  # sim: 0.0172

t_servo = 0.085883  # time constant of servo

# concatenate the parameters to make a new list
# fmt: off
physical_param_list = [
    mass, gravity, Ixx, Iyy, Izz,
    kq_d_kt,
    dr1, p1_b[0], p1_b[1], p1_b[2],
    dr2, p2_b[0], p2_b[1], p2_b[2],
    dr3, p3_b[0], p3_b[1], p3_b[2],
    0.0, t_servo,
]
# fmt: on

physical_param_list.extend([0.0, 0.0, 0.0])
physical_param_list.extend([1.0, 0.0, 0.0, 0.0])  # to compatible with end-effectors.
