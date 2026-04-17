FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV ROS_DOMAIN_ID=0

RUN apt-get update && apt-get upgrade -y && apt-get install -y -qq \
    apt-utils \
    python3-colcon-common-extensions \
    python3-pip \
    python3-numpy \
    git \
    curl \
    lsb-release \
    build-essential \
    cmake \
    ros-${ROS_DISTRO}-nav-msgs \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-control-toolbox \
    ros-${ROS_DISTRO}-mavros \
    # Nav2 packages
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-nav2-msgs \
    ros-${ROS_DISTRO}-nav2-common \
    ros-${ROS_DISTRO}-nav2-util \
    ros-${ROS_DISTRO}-nav2-controller \
    ros-${ROS_DISTRO}-nav2-planner \
    ros-${ROS_DISTRO}-nav2-behaviors \
    ros-${ROS_DISTRO}-nav2-bt-navigator \
    ros-${ROS_DISTRO}-nav2-velocity-smoother \
    ros-${ROS_DISTRO}-nav2-lifecycle-manager \
    ros-${ROS_DISTRO}-nav2-navfn-planner \
    ros-${ROS_DISTRO}-nav2-regulated-pure-pursuit-controller \
    ros-${ROS_DISTRO}-rtabmap-ros \
    ros-${ROS_DISTRO}-rtabmap-odom \
    ros-${ROS_DISTRO}-rtabmap-slam \
    ros-${ROS_DISTRO}-rtabmap-msgs \
    # Perception + TF packages
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
    ros-${ROS_DISTRO}-std-srvs \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-ros2cli \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-depthimage-to-laserscan \
    # Build tools
    python3-pytest \
    python3-rosdep \
    python3-vcstool \
    python3-catkin-pkg \
    python3-ament-package \
    # C++ libraries
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

RUN rosdep update

RUN pip3 install flask flask-cors flask-socketio python-socketio eventlet pyserial ultralytics


RUN mkdir src
COPY . /src
