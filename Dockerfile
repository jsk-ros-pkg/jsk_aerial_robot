# For use on Jetson Orin NX with ARM64 architecture
FROM arm64v8/ros:noetic

# Use bash as default shell
SHELL ["/bin/bash", "-c"]

# Use NVIDIA L4T base image for Jetson compatibility
# FROM nvcr.io/nvidia/l4t-pytorch:r36.2.0-pth2.1-py3

ENV ROS_DISTRO=noetic
ENV DEBIAN_FRONTEND=noninteractive
# For NVIDIA container toolkit
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility

# Install apt packages
RUN apt-get update && \
    apt-get install -y \
    build-essential \
    python3-wstool \
    python3-catkin-tools \
    python3-pip \
    git \
    curl \
    nano \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install \
    # Avoid warning with 68.1.0
    setuptools==45.2.0 \
    cmake==3.27 \
    ninja==1.13 \
    numpy==1.24.4 \
    pandas==1.3.5 \
    rospkg==1.3.0 \
    # Get stable version (==2.6.0 not compatible with Ubuntu 20.04)
    torch==2.4.1 \
    torchsummary==1.5.1 \
    # Get stable version (==1.15.3 not compatible with Ubuntu 20.04)
    scipy==1.10.0 \
    scikit-build==0.18.1 \
    # Get stable version (==1.6.1 not compatible with Ubuntu 20.04)
    scikit-learn==1.3.2 \
    casadi==3.7.0 \
    # Get stable version (==2025.1.1 not compatible with Ubuntu 20.04)
    transformations==2022.9.26 \
    pyquaternion==0.9.9 \
    pyyaml==5.4.1
    # For real-time inference
    # tensorrt \
    # pycuda

# Set up acados
RUN cd /root && \
    git clone https://github.com/acados/acados.git --branch v0.5.0 && \
    cd /root/acados && \
    git submodule update --recursive --init && \
    mkdir -p /root/acados/build && \
    cd /root/acados/build && \
    cmake -DACADOS_WITH_QPOASES=ON .. && \
    make install -j4
# acados Python interface
RUN pip3 install -e /root/acados/interfaces/acados_template
RUN echo '\n# acados' >> /root/.bashrc
RUN echo 'export ACADOS_SOURCE_DIR=/root/acados' >> /root/.bashrc
RUN echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/root/acados/lib' >> /root/.bashrc
RUN source /root/.bashrc
# Download and install t_renderer binary for acados
# Install rust (and cargo) for building from source
RUN cd /root/acados && \
    curl https://sh.rustup.rs -sSf | sh -s -- -y && \
    git clone https://github.com/acados/tera_renderer.git --branch v0.0.35 && \
    cd /root/acados/tera_renderer && \
    /root/.cargo/bin/cargo build --verbose --release && \
    cp /root/acados/tera_renderer/target/release/t_renderer /root/acados/bin

# Set up workspace, install ROS and its dependencies (workaround for rospkg OS import issue)
RUN apt-get update && \
    apt-get install -y python3-rosdep && \
    apt purge -y python3-rospkg && \
    pip uninstall -y rospkg && \
    apt-get install -y python3-rospkg && \
    rosdep init || true && \
    rosdep update --include-eol-distros

RUN mkdir -p /root/ros/jojo_ws/src && \
    cd /root/ros/jojo_ws/src && \
    git clone https://github.com/johanneskbl/jsk_aerial_robot.git -b develop/neural_MPC && \
    cd /root/ros/jojo_ws && \
    wstool init src && \
    wstool merge -t src src/jsk_aerial_robot/aerial_robot_noetic.rosinstall && \
    wstool update -t src && \
    rosdep install -y -r --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}

# RUN cd /root/ros/jojo_ws && \
#     catkin config --extend /opt/ros/${ROS_DISTRO} && \
#     catkin build

RUN echo '\n# ROS' >> /root/.bashrc
RUN echo 'source /root/ros/jojo_ws/devel/setup.bash' >> /root/.bashrc
RUN echo "alias roslaunch='roslaunch 2> >(grep -v TF_REPEATED_DATA >&2)' # avoid annoying warning messages" >> /root/.bashrc
RUN echo "alias rosrun='rosrun 2> >(grep -v TF_REPEATED_DATA >&2) 2> >(grep -v buffer_core >&2) 2> >(grep -v TF_OLD >&2)' # avoid annoying warning messages" >> /root/.bashrc

RUN echo '\n# Workspace' >> /root/.bashrc
RUN echo "alias jsk_aerial_robot='cd /root/ros/jojo_ws/src/jsk_aerial_robot'" >> /root/.bashrc

WORKDIR /root/ros/jojo_ws
CMD ["bash"]
