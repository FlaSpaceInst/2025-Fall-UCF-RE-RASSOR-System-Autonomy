cmake_minimum_required(VERSION 3.8)
project(re_rassor_mission_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ======================
# Dependencies
# ======================

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)       # Trigger service for /re_rassor/calibrate
find_package(nav_msgs REQUIRED)       # Odometry messages for wheel + visual fusion
find_package(nav2_msgs REQUIRED)      # NavigateToPose action
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)        # TransformBroadcaster + StaticTransformBroadcaster
find_package(tf2_geometry_msgs REQUIRED)

set(DEPS
  rclcpp
  rclcpp_action
  geometry_msgs
  std_msgs
  std_srvs
  nav_msgs
  nav2_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# ======================
# mission_control node
# ======================

add_executable(mission_control src/mission_control.cpp)
ament_target_dependencies(mission_control ${DEPS})

# ======================
# navigate_cli tool
# ======================

add_executable(navigate_cli src/navigate_cli.cpp)
ament_target_dependencies(navigate_cli ${DEPS})

# ======================
# Install
# ======================

install(TARGETS
  mission_control
  navigate_cli
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()