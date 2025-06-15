# Generate c code for NMPC controller

To create, assemble and build the acados OCP model and solver run each python file individually with

```
python gen_nmpc_code.py <class_name_of_controller>
```

To see the full ist of available controllers, run

```
python gen_nmpc_code.py -h
```

The resulting c files are stored in
```aerial_robot_control/include/aerial_robot_control/nmpc/nmpc_tilt_mt/tilt_xx/name_of_controller```. They are incorporated in
the rest of the ```aerial_robot``` framework.

Further developer tools are set up, for example a simulation and visualization environment.

# Naming Rule

The naming rule of the NMPC model is as follows:

```
[tilt/fix]_[number of rotors]_[NMPC model speciality]_w_[disturbance on CoG]_[drag on each rotor]
```

For example,

```
tilt_qd_servo_w_dist_drag
```

In the future:

```
tilt_qd_servo_w_cog_end_dist
```

The model is unique, but different cog and end disturbances share a same interface.