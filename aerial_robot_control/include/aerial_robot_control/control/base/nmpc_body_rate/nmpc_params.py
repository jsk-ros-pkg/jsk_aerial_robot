#!/usr/bin/env python
# -*- encoding: ascii -*-
import fhnp_params as QD

gravity = QD.gravity
mass = QD.mass

# basic params
N_node = 20
T_horizon = 2
ts_nmpc = 0.02  # 100 Hz
th_pred = T_horizon / N_node  # seconds

n_states = 10
n_controls = 4

# params for constraints
# constraint
w_max = 6  # 1 rad/s ~ 57 deg/s
w_min = -6
c_max = QD.c_max
c_min = 0

v_max = 20  # TODO: set small in real flight
v_min = -20

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
long_list_size = int(th_pred * N_node / ts_nmpc) + 1
if th_pred * N_node / ts_nmpc - int(th_pred * N_node / ts_nmpc) > 1e-6:
    raise ValueError("please check: th_pred must be an integer multiple of th_nmpc")
xr_list_index = slice(0, long_list_size, int(th_pred / ts_nmpc))
