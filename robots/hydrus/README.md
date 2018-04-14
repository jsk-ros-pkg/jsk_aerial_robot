# HYDRUS: 

definiation: Transformable multirotor with two-dimensional multilinks

paper: https://www.tandfonline.com/doi/full/10.1080/01691864.2016.1181006

# how to use:

## standard command:

### real machine
- Realtime operation for quad with pos control with mocap:
```
$ roslaunch hydrus hydrusx_bringup.launch control_mode:=0 estimate_mode:=1 type:=quad
```

### simulation
- Simulation for quad with pos control:
```
$ roslaunch hydrus hydrusx_bringup.launch real_machine:=false simulation:=true headless:=false control_mode:=0 type:=quad
```

