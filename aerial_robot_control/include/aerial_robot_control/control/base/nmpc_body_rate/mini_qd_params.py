#!/usr/bin/env python
# -*- encoding: ascii -*-
import numpy as np

######################################################################################
#  mini quadrotor
######################################################################################

mass = 0.656400  # 1.4844  # kg .  add realsense and gps modules: 1.5344 kg; pure aircraft: 1.4844 kg
l_frame = 0.077396  # m
alpha_frame = 45.0 * np.pi / 180.0  # rad
gravity = 9.81  # m/s^2
Jx = 0.002523  # 0.0094  # kg m^2
Jy = 0.002046  # 0.0134  # kg m^2
Jz = 0.003927  # 0.0145  # kg m^2
Jxz = 0.000042  # 0.0 # kg m^2

# max collective acceleration
c_max = 7.5 * 4 / mass  # N/kg

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
