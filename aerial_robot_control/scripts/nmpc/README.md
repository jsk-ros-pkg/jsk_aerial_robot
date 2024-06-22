# Naming Rule

The naming rule of the NMPC model is as follows:

```
[tilt/fix]_[rotor number]_[NMPC model specialty]_w_[disturbance on CoG]_[drag on each rotor]
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