# Beetle-Art-Omni

## Installation

### 1. Install `acados`

The version of `acados` should be aligned with the version of `3rdparty/acados/CMakeLists.txt` -> `GIT_TAG`.

Specifically, clone the branch for **v0.5.0** from the `acados` git:
```bash
git clone https://github.com/acados/acados.git --branch v0.5.0
```
Then, follow the instructions below:
- Install acados itself: Please follow the instructions on the acados website https://docs.acados.org/installation/index.html
- Install Python interface: Please follow the instructions on the acados website https://docs.acados.org/python_interface/index.html, but don't create virtual env in step 2. The virtual env has compatibility problem with ROS env.
- **Pay attention** that you must execute the step 5 in https://docs.acados.org/python_interface/index.html to test the installation. This step should automatically install t_renderer. If something goes wrong, please follow step 6 to manually install the t_renderer binary.
- When performing step 6, please note that VIM4 is **aarch64(arm64)**, don't build t_renderer in amd64(x86) format. Please see the build instruction below.
- Optional: For developers trying to optimize the solver options for their applications, here is a convenient command to install more tested solvers in the cmake process of acados:
```bash
cmake -DACADOS_WITH_QPOASES=ON -DACADOS_WITH_DAQP=ON -DACADOS_WITH_QPDUNES=ON -DACADOS_WITH_OSQP=ON ..
```

[VIM4] Build t_renderer from source for aarch64(arm64):
```bash
git clone https://github.com/acados/tera_renderer.git --branch v0.0.34
cd tera_renderer
# Install rust, cargo etc.
sudo apt install cargo
cargo build --verbose --release
```
If you meet the problem that the version of rustc is too low (given by ChatGPT):
```bash
which rustc  # if it's /usr/bin/rustc, it's too old
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh  # install rustup, as recommended
source ~/.cargo/env
rustup update stable
rustc --version  # check the version, should be >=1.84.
````

(optional) Add SSH key to github account for easier cloning: please ask ChatGPT for help if you don't know how to do this.

### 2. Install the code base and the necessary ROS related packages ...

Setup the folder architecture and clone the repo **with the specific branch**:

```bash
mkdir -p ~/[path_to_ws]/src
cd ~/[path_to_ws]/src
# Recommended: clone including git submodules
git clone --recurse-submodules https://github.com/Li-Jinjie/jsk_aerial_robot_dev.git -b develop/MPC_tilt_mt    # pay attention to the branch flag

# If you already cloned without submodules, run these after cloning:
# cd jsk_aerial_robot_dev
# git submodule update --init --recursive
```

### 2.1 ... for Ubuntu 20.04 and ROS Noetic
Install ROS Noetic for Ubuntu 20.04 from https://wiki.ros.org/ROS/Installation and source the setup file:

```bash
source /opt/ros/noetic/setup.bash
```

We use rosdep to manage the dependencies. So, if you have never done this in your computer before, do the following:

```bash
sudo rosdep init
rosdep update
```

Then, do the following:
```bash
cd ~/[path_to_ws]
wstool init src
wstool merge -t src src/jsk_aerial_robot_dev/aerial_robot_noetic.rosinstall
wstool update -t src    # install unofficial packages
rosdep install -y -r --from-paths src --ignore-src --rosdistro noetic   # install the dependencies/packages stated in package.xml
```

### 2.2 ... for Ubuntu 22.04 and ROS-O
Run the following bash script that conveniently installs all packages and software, including ROS-O for Ubuntu 22.04 from https://ros.packages.techfak.net/. Then source the setup file:

```bash
./jsk_aerial_robot_dev/configure.sh   # for configuration especially for ROS-O in jammy
source /opt/ros/one/setup.bash  # or .zsh if you are using zsh
```

We use rosdep to manage the dependencies. So, if you have never done this in your computer before, do the following:

```bash
sudo rosdep init
rosdep update
```

Then, do the following:

```bash
cd ~/[path_to_ws]
wstool init src
wstool merge -t src src/jsk_aerial_robot_dev/aerial_robot_${ROS_DISTRO}.rosinstall
wstool update -t src    # install unofficial packages
rosdep install -y -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO      # install the dependencies/packages stated in package.xml
```

### 3. Install python packages and link them to acados
Install required packages:
```bash
pip install -r src/jsk_aerial_robot_dev/aerial_robot_control/scripts/requirements.txt
```
For VIM4, since  it's Ubuntu20.04 python3.8, pandas's version is incompatible, we should use different pkgs:
```bash
pip install -r src/jsk_aerial_robot_dev/aerial_robot_control/scripts/requirementsVIM4.txt
```

For the first run, **uncomment** these code in `aerial_robot_control/scripts/nmpc/gen_nmpc_code_all.sh`
```bash
MODELS=(
    NMPCFixQdAngvelOut
    NMPCFixQdThrustOut
    NMPCTiltQdNoServo
    NMPCTiltQdServo
    NMPCTiltQdServoDist
    NMPCTiltQdServoImpedance
    NMPCTiltQdServoThrustDist
    NMPCTiltQdServoThrustImpedance
    NMPCTiltTriServo
    NMPCTiltBiServo
    NMPCTiltBi2OrdServo
    MHEWrenchEstAccMom
)
```

### 4. Build the workspace with `catkin`

```bash
cd ~/path_to_ws
catkin build
```

For convenience, open `~/.bashrc` and add sourcing of the workspace to the end of the file:

```bash
In ~/.bashrc:

source ~/[path_to_ws]/devel/setup.bash
```

#### Addendum: Potential fixes to common problems
- A frequent problem is the handling of the jobservers in the build process. When occuring during the build process - especially in the aerial_robot_control package - please try simply running the build command again.
- If for some reason there is a NumPy version error while building, the correct version to use is 1.21.5 since ROS-O only supports versions up to it. Therefore, please check with `apt list | grep numpy` and `pip list | grep numpy` for all NumPy versions on your system. Then uninstall all packages corresponding to incorrect versions using apt or pip respectively. Then reinstall NumPy with the correct version flag, e.g., `pip install numpy==1.21.5`.

### 5. If the build is successful, comment the code in step 3 back.

## Simulation

### 1. Start the simulation
Run the simulation with the following command:
```bash
roslaunch beetle_omni bringup_nmpc_omni.launch real_machine:=false simulation:=True headless:=False nmpc_mode:=0
```
### 2. Start the keyboard script
Run the keyboard with the following command:
```bash
rosrun aerial_robot_base keyboard_command.py
```
Then input `r` to arm the motors and input `t` to let the drone take off.

### 3. Send the trajectory
**After the main window printed 'Hovering'**, please run the following command to send a trajectory:
```bash
rosrun aerial_robot_planning mpc_smach_node.py beetle1
```
Then choose the trajectory you want the drone to perform.

## Experiments

Most commands are the same with simulation.

For using wrench sensor, call the following command in hovering to calibrate the wrench sensor:
```bash
rosservice call /cfs_sensor_calib "{}"
```

## Flying Hand

We need two notebooks, one as ground station and the other for visual feedback.

### MoCap Computer

1. Run the software for the data glove and calibrate it. Note that the ip should be set to the ros master computer.
2. Check the MoCap is set correctly.

If there is something wrong with glove connection, please refer
to https://stretchsense.my.site.com/defaulthelpcenter26Sep/s/article/Studio-Glove-and-Dongle-Setup?language=en_US

### Onboard Computer

Four terminals are needed.

Before takeoff, launch necessary files

1. `roslaunch beetle_omni bringup_nmpc_omni.launch nmpc_mode:=0 wrench_est_mode:=none  # This is the main launch file for the robot.`
2. `roslaunch aerial_robot_planning hand_arm_glove.launch`
3. `rosbag record -a`

After the robot reach the status of hovering - see terminal output of main launch file -,

4. Run `rosrun aerial_robot_planning mpc_smach_node.py beetle1`, and press 'h' to enter the hand control mode. Note that
   we need to wear the hand part, the shoulder part, and gloves to control the robot.
5. To modify the `/hand/control_mode` parameter using `rosparam` in the terminal, run the following command:
    `rosparam set /hand/control_mode <mode_value>`
    Where `<mode_value>` can be set as one of the following:
    - **1**: Operation Mode
    - **2**: Spherical Coordinate Mode
    - **3**: Cartesian Coordinate Mode
    - **4**: Lock Mode
    - **5**: Exit Mode

    To check the current mode, you can run:`rosparam get /operation_mode`

### Ground Station
Three terminals are needed.

Before takeoff:

1. run `sudo ds4drv` to connect the joystick.
2. `roslaunch beetle_omni bringup_pc.launch`

### Visual Computer

1. `rosrun aerial_robot_planning visual_fb_mode.py`
2. Adjust the window to fullfill the screen.

## Build on onboard computer
If you run into dependency issues with GLIBC - e.g., because you use Ubuntu 20.04 to run ROS but still need to use acados for NMPC control - please build `tera_renderer` from source. For background, GLIBC is one of the most central libraries that the Linux system builds on and its version is highly connected to the Ubuntu version used. Therefore, it is hard to upgrade it when needed by certain packages. One of these packages that need a relatively high version of GLIBC is `tera_renderer`. The newest version - at the time of writing - is v0.2.0 which provides already build AMD64 and ARM64 executables. To run these executables a GLIBC of >= 2.32 but Ubuntu 20.04 is on version GLIBC 2.31. The most recommended solution is to build the `tera_renderer` executable from source based on a previous version of the Tera Renderer git. To upgrade GLIBC is widely discouraged as it can easily break the OS.

To install `tera_renderer` from source execute this _after_ cloning and installing acados.

```bash
# Install rust, cargo etc.
curl https://sh.rustup.rs -sSf | sh -s -- -y

# Clone the Tera Renderer git and use a previous version that supports the GLIBC of the Ubuntu version you are using
git clone https://github.com/acados/tera_renderer.git --branch v0.0.35

cd <acados_root>/tera_renderer

# Build the executable using cargo and store it in the folder 'release'
$HOME/.cargo/bin/cargo build --verbose --release

# Move binary to folder such that acados can access it
cp <acados_root>/tera_renderer/target/release/t_renderer <acados_root>/bin
```
