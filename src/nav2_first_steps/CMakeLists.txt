cmake_minimum_required(VERSION 3.8)
project(nav2_first_steps)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Build nav2_client executable
add_executable(nav2_client src/nav2_client.cpp)
ament_target_dependencies(nav2_client
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  std_msgs #added from here
  tf2
  tf2_geometry_msgs
)

# Build initial_pose_pub executable
add_executable(initial_pose_pub src/initial_pose_pub.cpp)
ament_target_dependencies(initial_pose_pub
  rclcpp
  geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install executables
install(TARGETS
  nav2_client
  initial_pose_pub
  mission_control #added
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
