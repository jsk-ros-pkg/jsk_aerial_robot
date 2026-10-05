# 2025-10-08 copy parameters to be independent of any ROS package
# these parameters are used in GitHub action to ensure the correctness of the nmpc code
# In real usage, these parameters will be reloaded from URDF.

mass = 1.44441  # sim: 1.44441 # kg
gravity = 9.798  # sim: 9.80665 # m/s^2   Tokyo 9.798; in sim: 9.80665
Ixx = 0.055429
Iyy = 0.017873
Izz = 0.067851
dr1 = 1
p1_b = [0.0, 0.2375, 0.12098]
dr2 = -1
p2_b = [0, -0.2375, 0.12098]
kq_d_kt = 0.0172

# servo parameters
## 1-ord
t_servo = 0.15858  # time constant of servo

## 2-ord
kps = 72.7570
kds = 11.5368  # kds has included the damping ratio

i_sxx = 0.0036697  # 0.0036697 calculated from UDRF

# concatenate the parameters to make a new list
# fmt: off
physical_param_list = [
    mass, gravity, Ixx, Iyy, Izz,
    kq_d_kt,
    dr1, p1_b[0], p1_b[1], p1_b[2],
    dr2, p2_b[0], p2_b[1], p2_b[2],
    0.0, t_servo,
]
# fmt: on

physical_param_list.extend([0.0, 0.0, 0.0])
physical_param_list.extend([1.0, 0.0, 0.0, 0.0])  # to compatible with end-effectors.
