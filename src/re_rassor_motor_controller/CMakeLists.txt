cmake_minimum_required(VERSION 3.8)
project(re_rassor_motor_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(std_srvs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(re_rassor_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(CURL REQUIRED)
find_package(nlohmann_json REQUIRED)  # Added for JSON support

# Include directories
include_directories(include)

# ── motor_controller (HTTP-based, original) ───────────────────────────────────
add_executable(motor_controller
  src/motor_controller.cpp
)

target_include_directories(motor_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(motor_controller
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  re_rassor_interfaces
  tf2
  tf2_geometry_msgs
  std_srvs
)

target_link_libraries(motor_controller
  CURL::libcurl
  nlohmann_json::nlohmann_json
)

# ── serial_motor_controller (serial-hardware, same interface) ─────────────────
# Compile as a library first so the test target can link against it.
add_library(serial_motor_controller_lib
  src/serial_motor_controller.cpp
)
target_include_directories(serial_motor_controller_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(serial_motor_controller_lib
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  re_rassor_interfaces
  tf2
  tf2_geometry_msgs
  std_srvs
)

add_executable(serial_motor_controller
  src/serial_motor_controller_main.cpp
)
target_include_directories(serial_motor_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(serial_motor_controller serial_motor_controller_lib)

# Install include directory
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Install executables and library
install(TARGETS
  motor_controller
  serial_motor_controller
  serial_motor_controller_lib
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # C++ unit tests (odometry math, velocity clamping, routine flags)
  ament_add_gtest(test_motor_controller_cpp
    test/test_motor_controller.cpp
  )
  target_include_directories(test_motor_controller_cpp PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  )

  # C++ Arduino simulation tests — verify serial bytes via pty
  ament_add_gtest(test_serial_arduino_sim
    test/test_serial_arduino_sim.cpp
  )
  target_include_directories(test_serial_arduino_sim PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  )
  ament_target_dependencies(test_serial_arduino_sim
    rclcpp
    std_msgs
    nav_msgs
    geometry_msgs
    re_rassor_interfaces
    tf2
    tf2_geometry_msgs
  )
  target_link_libraries(test_serial_arduino_sim serial_motor_controller_lib util)  # util for openpty()

  # Python integration tests
  ament_add_pytest_test(test_motor_controller_py
    test/test_motor_controller.py
    TIMEOUT 60
  )
endif()

ament_package()