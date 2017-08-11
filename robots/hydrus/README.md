# Usage
## hydrus3:
- Simulation:
```
$ roslaunch hydrus hydrus3_bringup.launch real_machine:=false simulation:=true headless:=false control_mode:=2 type:=quad
```

## hydrusx:
- Realtime operation for quad with pos control with mocap:
```
$ roslaunch hydrus hydrusx_bringup.launch control_mode:=0 estimate_mode:=1 type:=quad
```

- Simulation for quad with pos control:
```
$ roslaunch hydrus hydrusx_bringup.launch real_machine:=false simulation:=true headless:=false control_mode:=0 type:=quad
```

