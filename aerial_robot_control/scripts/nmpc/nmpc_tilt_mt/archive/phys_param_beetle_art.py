# 2025-10-08 copy parameters from PhysParamBeetleArt.yaml to be independent of beetle package
# these parameters are used in GitHub action to ensure the correctness of the nmpc code
# In real usage, these parameters will be reloaded from URDF.

mass = 2.773
gravity = 9.798
Ixx = 0.04170
Iyy = 0.03945
Izz = 0.07068
dr1 = 1
p1_b = [0.137712, 0.137882, 0.0297217]
dr2 = -1
p2_b = [-0.137745, 0.137882, 0.0297217]
dr3 = 1
p3_b = [-0.137745, -0.138284, 0.0297217]
dr4 = -1
p4_b = [0.137712, -0.138284, 0.0297217]
kq_d_kt = 0.0153

t_servo = 0.085883  # Time constant of servo
t_rotor = 0.0942  # Time constant of rotor

c0 = -0.00278
c1 = -0.02147
c2 = 0.08134
c3 = 0.00470
c4 = -0.02439

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

physical_param_list.extend([0.0, 0.0, 0.0])
physical_param_list.extend([1.0, 0.0, 0.0, 0.0])  # to compatible with end-effectors.
