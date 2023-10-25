#!/usr/bin/env python
# -*- encoding: ascii -*-
import numpy as np

######################################################################################
#  fhnp
######################################################################################

mass = 1.4844  # kg .  add realsense and gps modules: 1.5344 kg; pure aircraft: 1.4844 kg
l_frame = 0.1372  # m
alpha_frame = 45.0 * np.pi / 180.0  # rad
gravity = 9.81  # m/s^2
Jx = 0.0094  # kg m^2
Jy = 0.0134  # kg m^2
Jz = 0.0145  # kg m^2
Jxz = 0.0

# max collective acceleration
c_max = gravity / 0.36  # TODO:  change to fc_max = 4.0 * k_t * (o_max**2)  # N


#   Propeller thrust / torque parameters
o_max = 24000 / 1000  # kRPM
o_min = 2600 / 1000  # kRPM

c_q = 3.7611e-10  # Nm/RPM^2
c_t = 2.8158e-08  # N/RPM^2

t_w_r = 4.3100  # thrust-to-weight ratio

######################################################################################
#   Calculation Variables
######################################################################################
#   gamma parameters pulled from page 36 (dynamics)
gamma = Jx * Jz - (Jxz**2)
gamma1 = (Jxz * (Jx - Jy + Jz)) / gamma
gamma2 = (Jz * (Jz - Jy) + (Jxz**2)) / gamma
gamma3 = Jz / gamma
gamma4 = Jxz / gamma
gamma5 = (Jz - Jx) / Jy
gamma6 = Jxz / Jy
gamma7 = ((Jx - Jy) * Jx + (Jxz**2)) / gamma
gamma8 = Jx / gamma
