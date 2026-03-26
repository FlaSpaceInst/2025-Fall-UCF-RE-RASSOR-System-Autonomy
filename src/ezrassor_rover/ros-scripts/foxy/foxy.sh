#!/bin/bash

# set locale
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

# set sources
sudo apt install software-properties-common
sudo add-apt-repository universe

# set GPG keys
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# add the repo sources
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# gen updates
sudo apt update
sudo apt upgrade

# install ros
sudo apt install ros-foxy-ros-base python3-argcomplete
sudo apt install ros-dev-tools

# # source command
# # Replace ".bash" with your shell if you're not using bash
# # Possible values are: setup.bash, setup.sh, setup.zsh
# source /opt/ros/foxy/setup.bash
# export to .bashrc
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc