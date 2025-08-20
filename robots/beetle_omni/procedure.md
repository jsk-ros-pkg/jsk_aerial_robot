1. Omnidirectional

Install ball.

```bash
roslaunch beetle_omni bringup_nmpc_omni.launch nmpc_mode:=0 wrench_est_mode:=none end_effector:=ball
```

Traj: 6, 10, 17, 18

2. Singular points

```bash
roslaunch beetle_omni bringup_nmpc_omni.launch nmpc_mode:=0 wrench_est_mode:=none end_effector:=ball
```

traj: 15

3. Effector-Centric

urdf:     <joint name="ball_to_ee_contact" type="fixed">
<parent link="ball"/>
<child link="ee_contact"/>
<origin xyz="0 0 0.11" rpy="0 0 0"/>
</joint>

0 0 0.11: traj: 7 + poking
0 0 -0.14: traj: 7 + poking
0.2 0.0 0.11: traj: 7
-0.3 0.0 0.11: traj: 7

```bash
roslaunch beetle_omni bringup_nmpc_omni.launch nmpc_mode:=0 wrench_est_mode:=none end_effector:=ball
```

4. Wrench Estimation --- Disk

Install disk.

1)

```bash
roslaunch beetle_omni bringup_nmpc_omni.launch nmpc_mode:=0 wrench_est_mode:=acc end_effector:=disk
```

Tilt 90 deg, then using a poke with a flat surface to push it. More than 30 N?

2)

```bash
roslaunch beetle_omni bringup_nmpc_omni.launch nmpc_mode:=0 wrench_est_mode:=none end_effector:=disk
```

Tilt 90 deg, then using a poke with a flat surface to push it. More than 30 N?

3)

```bash
roslaunch beetle_omni bringup_nmpc_omni.launch nmpc_mode:=0 wrench_est_mode:=acc end_effector:=disk
```

Hand-based control, push the door.

4)

```bash
roslaunch beetle_omni bringup_nmpc_omni.launch nmpc_mode:=0 wrench_est_mode:=none end_effector:=disk
```

Hand-based control, push the door.

4. Wrench Estimation --- Valve

1)

```bash
roslaunch beetle_omni bringup_nmpc_omni.launch nmpc_mode:=0 wrench_est_mode:=acc end_effector:=disk
```

Tilt 90 deg, then using a valve to rotate it.

2)

```bash
roslaunch beetle_omni bringup_nmpc_omni.launch nmpc_mode:=0 wrench_est_mode:=none end_effector:=disk
```

Tilt 90 deg, then using a valve to rotate it.

3) continuous rotating the valve?

Fix the y and z axis. Activate the wrench estimation in yaw torque.
