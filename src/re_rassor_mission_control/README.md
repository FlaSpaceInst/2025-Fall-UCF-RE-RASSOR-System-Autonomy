# Create the package directory in your ROS2 workspace
cd ~/ros2_ws/src
mkdir -p ezrassor_mission_control/src

# Copy the files to their locations
# (Copy the contents above into the respective files)

# Build
cd ~/ros2_ws
colcon build --packages-select ezrassor_mission_control

# Source
source install/setup.bash

# Test
ros2 run ezrassor_mission_control mission_control \
  --ros-args \
  -p goal_x:=5.0 \
  -p goal_y:=3.0 \
  -p goal_theta:=1.57
