# Mini Quadrotor

# How to use

In one terminal, run

`roslaunch mini_quadrotor bringup.launch real_machine:=false simulation:=True headless:=False`

In another terminal, run

`rosrun aerial_robot_base keyboard_command.py`

In this terminal, input `r` to arm the quadrotor, then input `t` to takeoff. The quadrotor will takeoff and hover.

Input `l` to land the quadrotor.

# How to use MPC

First, please follow the instruction in [here](https://docs.acados.org/installation/index.html) to install acados.

Then, in one terminal, run

`roslaunch mini_quadrotor bringup_nmpc.launch real_machine:=false simulation:=True headless:=False`

In another terminal, run

`rosrun aerial_robot_base keyboard_command.py`

In this terminal, input `r` to arm the quadrotor, then input `t` to takeoff. The quadrotor will takeoff and hover.

Input `l` to land the quadrotor.
