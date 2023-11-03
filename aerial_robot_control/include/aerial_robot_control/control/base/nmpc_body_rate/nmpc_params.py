#!/usr/bin/env python
# -*- encoding: ascii -*-
import mini_qd_params as QD

gravity = QD.gravity
mass = QD.mass

# basic params

T_pred = 2.0  # seconds
T_integ = 0.1  # seconds
T_samp = 0.025  # 40 Hz
N_node = int(T_pred / T_integ)

n_states = 10
n_controls = 4

# params for constraints
# constraint
w_max = 6  # 1 rad/s ~ 57 deg/s
w_min = -6
c_max = QD.c_max
c_min = 0

v_max = 1
v_min = -1

# params for the cost function
Qp_xy = 300  # 0
Qp_z = 400  # 500
Qv_xy = 10  # 0
Qv_z = 10  # 100
Qq_xy = 10  # 400
Qq_z = 100
Rw = 10  # 10
Rc = 5

# params for nmpc pt_pub
# note that the nmpc pt_pub construct a long list, so that each iteration the pt_pub only needs to calculate
# the first points and pop the last. This can save a lot of time comparing with for loop.
long_list_size = int(T_integ * N_node / T_samp) + 1
if T_integ * N_node / T_samp - int(T_integ * N_node / T_samp) > 1e-6:
    raise ValueError("please check: th_pred must be an integer multiple of th_nmpc")
xr_list_index = slice(0, long_list_size, int(T_integ / T_samp))
