cmake_minimum_required(VERSION 3.8)
project(re_rassor_traffic_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(re_rassor_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Add executable
add_executable(motor_controller
  src/motor_controller.cpp
)

ament_target_dependencies(motor_controller
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  re_rassor_interfaces
  tf2
  tf2_geometry_msgs
)

# Install targets
install(TARGETS
  motor_controller
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
