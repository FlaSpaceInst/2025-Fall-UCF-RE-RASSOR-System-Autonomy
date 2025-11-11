cmake_minimum_required(VERSION 3.8)
project(re_rassor_mission_control)

# C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(re_rassor_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_cmake_pytest REQUIRED)
find_package(launch_testing_ament_cmake REQUIRED)

include_directories(
  include
)

add_executable(re_rassor_mission_control src/mission_control.cpp)

ament_target_dependencies(re_rassor_mission_control
  rclcpp
  re_rassor_interfaces
  std_msgs
)

install(TARGETS
  re_rassor_mission_control
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
