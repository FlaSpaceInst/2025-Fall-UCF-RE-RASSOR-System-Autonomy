cmake_minimum_required(VERSION 3.8)
project(ezrassor_mission_control)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Create executable
add_executable(mission_control src/mission_control.cpp)

# Specify dependencies
ament_target_dependencies(mission_control
  rclcpp
  rclcpp_action
  geometry_msgs
  std_msgs
  nav2_msgs
  tf2
  tf2_geometry_msgs
)

# Install executable
install(TARGETS
  mission_control
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files if you add any
# install(DIRECTORY
#   launch
#   DESTINATION share/${PROJECT_NAME}/
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()