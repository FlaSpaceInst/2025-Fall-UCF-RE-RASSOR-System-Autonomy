FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
ENV ROS_DOMAIN_ID=0

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-pip \
    python3-numpy \
    python3-flask \
    python3-requests \ 
    git \
    curl \
    libcurl4-openssl-dev \
    nlohmann-json3-dev \
    libgeographiclib-dev \
    lsb-release \
    build-essential \
    ros-${ROS_DISTRO}-nav-msgs \
    ros-${ROS_DISTRO}-nav2-msgs \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-control-toolbox \
    ros-${ROS_DISTRO}-mavros \
    && rm -rf /var/lib/apt/lists/*


# Create workspace
WORKDIR /ros2_ws
RUN mkdir src

# Copy your package
COPY . src/re_rassor_motor_controller

# Build workspace
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install

# Source ROS 2 and workspace on container start
SHELL ["/bin/bash", "-c"]
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && exec bash"]
