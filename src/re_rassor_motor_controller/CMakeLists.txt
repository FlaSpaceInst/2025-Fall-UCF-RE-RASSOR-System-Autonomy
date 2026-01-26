cmake_minimum_required(VERSION 3.8)
project(re_rassor_motor_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(re_rassor_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(include)

# Add executable
add_executable(motor_controller
  src/motor_controller.cpp
)

# Set include directories for the target
target_include_directories(motor_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# Link dependencies
ament_target_dependencies(motor_controller
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  re_rassor_interfaces
  tf2
  tf2_geometry_msgs
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Install targets
install(TARGETS
  motor_controller
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # C++ unit tests
  ament_add_gtest(test_motor_controller_cpp
    test/test_motor_controller.cpp
  )
  target_include_directories(test_motor_controller_cpp PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  )

  # Python integration tests
  ament_add_pytest_test(test_motor_controller_py
    test/test_motor_controller.py
    TIMEOUT 60
  )
endif()

ament_package()