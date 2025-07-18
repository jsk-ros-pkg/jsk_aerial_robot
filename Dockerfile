# For use on Jetson Orin NX with ARM64 architecture
FROM arm64v8/ros:noetic

ENV ROS_DISTRO noetic
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y \
    python3-wstool \
    python3-catkin-tools \
    git \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN apt-get update && \
    apt-get install -y \
    python3-pip \
    python3-setuptools \
    python3-opencv

RUN pip3 install --no-cache-dir \
    torch==2.6.0 \
    scienceplots==2.1.1 \
    scipy==1.10 \
    scikit-learn==1.6.1 \
    matplotlib==3.5.1 \
    pandas==1.3.5 \
    casadi==3.7.0 \
    tikzplotlib==0.10.1 \
    scikit-learn==0.23.2 \
    progress-table==3.1.2 \
    torchsummary==1.5.1

# RUN apt-get update && \
#     apt-get install -y python3-rosdep && \
#     rosdep init || true && \
#     rosdep update --include-eol-distros

# RUN mkdir -p /root/ros/jsk_aerial_robot_ws/src && \
#     cd /root/ros/jsk_aerial_robot_ws && \
#     wstool init src && \
#     wstool set -u -t src jsk_aerial_robot http://github.com/jsk-ros-pkg/jsk_aerial_robot --git -y && \
#     cd src/jsk_aerial_robot && \
#     bash configure.sh && \
#     cd /root/ros/jsk_aerial_robot_ws && \
#     wstool merge -t src src/jsk_aerial_robot/aerial_robot_noetic.rosinstall && \
#     wstool update -t src

# RUN cd /root/ros/jsk_aerial_robot_ws && \
#     rosdep install -y -r --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}

# RUN cd /root/ros/jsk_aerial_robot_ws && \
#     catkin config --extend /opt/ros/${ROS_DISTRO} && \
#     catkin build

# RUN echo "source /root/ros/jsk_aerial_robot_ws/devel/setup.bash" >> /root/.bashrc

WORKDIR /root/ros/jsk_aerial_robot_ws
CMD ["bash"]