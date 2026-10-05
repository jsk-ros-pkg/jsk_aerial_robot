# 2025-10-08 copy parameters to be independent of any ROS package
# these parameters are used in GitHub action to ensure the correctness of the nmpc code
# In real usage, these parameters will be reloaded from URDF.

mass = 3.0386
gravity = 9.798
Ixx = 0.0627958
Iyy = 0.0620796
Izz = 0.0948795
dr1 = 1
dr2 = -1
dr3 = 1
dr4 = -1
p1_b = [0.194824, 0.194652, -0.00368224]
p2_b = [-0.194837, 0.194652, -0.00368224]
p3_b = [-0.194837, -0.195008, -0.00368224]
p4_b = [0.194824, -0.195008, -0.00368224]
kq_d_kt = 0.0165

t_servo = 0.0480  # time constant of servo
t_rotor = 0.0942  # time constant of rotor

ball_effector_p = [0, 0, 0.264]
ball_effector_q = [1, 0, 0, 0]  # qw, qx, qy, qz

# concatenate the parameters to make a new list
# fmt: off
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

# Add ball effector parameters
physical_param_list.extend(ball_effector_p)
physical_param_list.extend(ball_effector_q)
