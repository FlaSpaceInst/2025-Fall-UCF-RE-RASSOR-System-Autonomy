FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV ROS_DOMAIN_ID=0

# Fix clock skew that causes apt failures in some build environments
RUN hwclock -s 2>/dev/null || true

# Core tools — apt-utils first to suppress debconf warnings
RUN apt-get update && apt-get install -y -qq \
    apt-utils \
    build-essential \
    cmake \
    git \
    curl \
    lsb-release \
    python3-colcon-common-extensions \
    python3-pip \
    python3-numpy \
    python3-pytest \
    python3-rosdep \
    python3-vcstool \
    python3-catkin-pkg \
    python3-ament-package \
    && rm -rf /var/lib/apt/lists/*

# ROS 2 Humble — base msgs and nav stack
RUN apt-get update && apt-get install -y -qq \
    ros-${ROS_DISTRO}-nav-msgs \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-std-srvs \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-nav2-msgs \
    ros-${ROS_DISTRO}-nav2-controller \
    ros-${ROS_DISTRO}-nav2-planner \
    ros-${ROS_DISTRO}-nav2-behaviors \
    ros-${ROS_DISTRO}-nav2-bt-navigator \
    ros-${ROS_DISTRO}-nav2-velocity-smoother \
    ros-${ROS_DISTRO}-nav2-lifecycle-manager \
    ros-${ROS_DISTRO}-nav2-navfn-planner \
    ros-${ROS_DISTRO}-nav2-regulated-pure-pursuit-controller \
    && rm -rf /var/lib/apt/lists/*

# ROS 2 Humble — SLAM and perception
RUN apt-get update && apt-get install -y -qq \
    ros-${ROS_DISTRO}-rtabmap-ros \
    ros-${ROS_DISTRO}-rtabmap-odom \
    ros-${ROS_DISTRO}-rtabmap-slam \
    ros-${ROS_DISTRO}-rtabmap-msgs \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-tf2-sensor-msgs \
    ros-${ROS_DISTRO}-tf2-eigen \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-image-transport-plugins \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-camera-info-manager \
    ros-${ROS_DISTRO}-depth-image-proc \
    ros-${ROS_DISTRO}-image-geometry \
    ros-${ROS_DISTRO}-image-publisher \
    ros-${ROS_DISTRO}-sensor-msgs-py \
    ros-${ROS_DISTRO}-rclcpp \
    ros-${ROS_DISTRO}-rclcpp-action \
    ros-${ROS_DISTRO}-rclcpp-components \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-ros2cli \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-depthimage-to-laserscan \
    ros-${ROS_DISTRO}-backward-ros \
    && rm -rf /var/lib/apt/lists/*

# C++ system libraries
RUN apt-get update && apt-get install -y -qq \
    libgflags-dev \
    libgoogle-glog-dev \
    libeigen3-dev \
    libopencv-dev \
    libusb-1.0-0-dev \
    libuvc-dev \
    nlohmann-json3-dev \
    libwebsockets-dev \
    libcurl4-openssl-dev \
    && rm -rf /var/lib/apt/lists/*

# Update rosdep (base image already ran rosdep init)
RUN rosdep update

# Python packages for the controller server
RUN pip3 install flask flask-cors flask-socketio python-socketio eventlet pyserial

WORKDIR /ros2_ws
RUN mkdir -p src

COPY . src/2025-Fall-UCF-RE-RASSOR-System-Autonomy/

SHELL ["/bin/bash", "-c"]
ENTRYPOINT ["bash", "-c", \
    "source /opt/ros/${ROS_DISTRO}/setup.bash && \
     [ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash; \
     exec bash"]
