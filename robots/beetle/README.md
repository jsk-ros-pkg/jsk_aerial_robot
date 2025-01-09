# Beetle-Art-Omni

## Installation

### 1. Install acados

The version of acados should be aligned with the version of 3rdparty/acados/CMakeLists.txt -> GIT_TAG.

Specifically, first:
```bash
git clone https://github.com/acados/acados.git --branch v0.3.3
```
Then, follow the instructions below:
- Install acados itself: Please follow the instructions on the acados website https://docs.acados.org/installation/index.html
- Install Python interface: Please follow the instructions on the acados website https://docs.acados.org/python_interface/index.html, but don't create virtual env in step 2. The virtual env has compatibility problem with ROS env.
- Pay attention that you must execute the step 5 in https://docs.acados.org/python_interface/index.html to test the installation. This step should automatically install t_renderer. If something wrong, please follow step 6 to manually install t_renderer.

### 2. install the ros related packages

### 2.1 for ubuntu 20.04 and ROS Noetic

```bash
source /opt/ros/noetic/setup.bash
mkdir -p ~/path_to_ws/src
cd ~/path_to_ws/src
```
We use rosdep to manage the dependencies. So, 
if you have never done this in your computer before, do the following:

```bash
sudo rosdep init
rosdep update
```

Then, do the following:

```bash
wstool init src
git clone git@github.com:Li-Jinjie/jsk_aerial_robot_dev.git -b develop/MPC_tilt_mt  # -b means the branch
wstool merge -t src src/jsk_aerial_robot_dev/aerial_robot_noetic.rosinstall
wstool update -t src  # install those unofficial packages
rosdep install -y -r --from-paths src --ignore-src --rosdistro noetic # install the dependencies, aka the packages in the package.xml
```

### 2.2 for ubuntu 22.04 and ROS-O

```bash
mkdir -p ~/path_to_ws/src
cd ~/path_to_ws/src
git clone https://github.com/Li-Jinjie/jsk_aerial_robot_dev.git -b develop/MPC_tilt_mt  # -b means the branch
```

Please install ROS-O for ubuntu 22.04:

```bash
./jsk_aerial_robot_dev/configure.sh # for configuration especially for ros-o in jammy
source /opt/ros/one/setup.bash
cd ..
```

We use rosdep to manage the dependencies. So,
if you have never done this in your computer before, do the following:

```bash
sudo rosdep init
rosdep update
```

Then, do the following:

```bash
wstool init src
wstool merge -t src src/jsk_aerial_robot_dev/aerial_robot_${ROS_DISTRO}.rosinstall
wstool update -t src
rosdep install -y -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

### 3. For the first run, uncomment these code in aerial_robot_control/CMakeLists.txt
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

### 4. catkin build the workspace.

```bash
cd ~/path_to_ws
catkin build
```

### 5. If the building is successful, comment the code in step 3.

## Run the simulation

### 1. Start the simulation
Run the simulation with the following command:
```bash
roslaunch beetle bringup_nmpc_omni.launch real_machine:=false simulation:=True headless:=False nmpc_mode:=0
```
### 2. Start the keyboard script
Run the keyboard with the following command:
```bash
rosrun aerial_robot_base keyboard_command.py
```
Then input 'r' to arm the robot and input 't' to takeoff the robot.

### 3. Send the trajectory
**After the main window printed 'Hovering'**, please run the following command to send a trajectory:
```bash
rosrun aerial_robot_planning mpc_pt_pub_node.py beetle1
```
Then choose the trajectory you want to send.
