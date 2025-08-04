# For use on Jetson Orin NX with ARM64 architecture
FROM arm64v8/ros:noetic

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
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install \
    numpy==1.21.5 \
    torch==2.4.1 \
    # TODO get stable version (==2.6.0 not compatible)
    scienceplots==2.1.1 \
    scipy==1.10 \
    scikit-learn==1.3.2 \
    #TODO get stable version (==1.6.1) \
    matplotlib==3.5.1 \
    pandas==1.3.5 \
    casadi==3.7.0 \
    tikzplotlib==0.10.1 \
    progress-table==3.1.2 \
    torchsummary==1.5.1 \
    transformations==2022.9.26
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
RUN echo 'LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/root/acados/lib' >> /root/.bashrc
RUN echo 'ACADOS_SOURCE_DIR=/root/acados' >> /root/.bashrc
RUN source /root/.bashrc
# Download and install t_renderer binary for acados
# Install rust (and cargo) for building from source
RUN curl https://sh.rustup.rs -sSf | sh -s -- -y
RUN git clone https://github.com/acados/tera_renderer.git --branch v0.0.35
RUN cd /root/acados/tera_renderer
RUN /root/.cargo/bin/cargo build --verbose --release
RUN cp /root/acados/tera_renderer/target/release/t_renderer /root/acados/bin

# Set up workspace, install ROS and its dependencies
RUN apt-get update && \
    apt-get install -y python3-rosdep && \
    rosdep init || true && \
    rosdep update --include-eol-distros

RUN mkdir -p /root/ros/jsk_aerial_robot_ws/src && \
    cd /root/ros/jsk_aerial_robot_ws/src && \
    git clone https://github.com/johanneskbl/jsk_aerial_robot.git -b develop/RTNMPC && \
    cd /root/ros/jsk_aerial_robot_ws && \
    wstool init src && \
    # wstool set -u -t src jsk_aerial_robot http://github.com/johanneskbl/jsk_aerial_robot --git -y && \
    wstool merge -t src src/jsk_aerial_robot/aerial_robot_noetic.rosinstall && \
    wstool update -t src && \
    rosdep install -y -r --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}

# RUN cd /root/ros/jsk_aerial_robot_ws && \
#     catkin config --extend /opt/ros/${ROS_DISTRO} && \
#     catkin build

# RUN echo "source /root/ros/jsk_aerial_robot_ws/devel/setup.bash" >> /root/.bashrc

RUN echo "Please build the workspace 'catkin build' and 'source /root/ros/jsk_aerial_robot_ws/devel/setup.bash'"

WORKDIR /root/ros/jsk_aerial_robot_ws
CMD ["bash"]
