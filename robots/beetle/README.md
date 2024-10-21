# Beetle-Art-Omni

## Installation

1. catkin build the workspace through the main README.md.
2. checkout the develop/MPC_tilt_mt branch.
3. install the acados manually.

The version of acados should be aligned with the version of 3rdparty/acados/CMakeLists.txt -> GIT_TAG.

Specifically, first:
```bash
git clone https://github.com/acados/acados.git --branch v0.3.3
```

Then, follow the instructions in the acados website https://docs.acados.org/installation/index.html to install the acados.

4. For the first run, uncomment these code in aerial_robot_control/CMakeLists.txt
```bash
set(ACADOS_PYTHON_SCRIPTS
        ${PROJECT_SOURCE_DIR}/scripts/nmpc/fix_qd/fix_qd_angvel_out.py
        ${PROJECT_SOURCE_DIR}/scripts/nmpc/fix_qd/fix_qd_thrust_out.py
        ${PROJECT_SOURCE_DIR}/scripts/nmpc/tilt_qd/tilt_qd_no_servo.py
        ${PROJECT_SOURCE_DIR}/scripts/nmpc/tilt_qd/tilt_qd_servo.py
        ${PROJECT_SOURCE_DIR}/scripts/nmpc/tilt_qd/tilt_qd_servo_dist.py
        ${PROJECT_SOURCE_DIR}/scripts/nmpc/tilt_qd/tilt_qd_servo_thrust_dist.py
        ${PROJECT_SOURCE_DIR}/scripts/nmpc/tilt_tri/tilt_tri_servo.py
        ${PROJECT_SOURCE_DIR}/scripts/nmpc/tilt_bi/tilt_bi_servo.py
        ${PROJECT_SOURCE_DIR}/scripts/nmpc/tilt_bi/tilt_bi_2ord_servo.py
)
```

5. catkin build the workspace again.

## Run the simulation

1. run the simulation with the following command:
```bash
roslaunch beetle bringup_nmpc_omni.launch real_machine:=false simulation:=True headless:=False nmpc_mode:=0
```
2. run the keyboard with the following command:
```bash
rosrun aerial_robot_base keyboard_command.py
```
Then input 'r' to arm the robot and input 't' to takeoff the robot.

3. After the main window printed 'Hovering', please run the following command to send a trajectory:
```bash
rosrun aerial_robot_planning mpc_pt_pub_node.py beetle1 3 -num 1
```
