cmake_minimum_required(VERSION 3.8)
project(re_rassor_obstacle_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ======================
# Dependencies
# ======================

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

find_package(CURL REQUIRED)

# ======================
# Depth + PointCloud Node
# ======================

add_executable(depth_costmap_node
  src/depth_costmap_node.cpp
)

ament_target_dependencies(depth_costmap_node
  rclcpp
  sensor_msgs
  nav_msgs
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
)

target_link_libraries(depth_costmap_node
  CURL::libcurl
)

# ======================
# Scan → Costmap Node
# ======================

add_executable(scan_to_costmap_node
  src/scan_to_costmap_node.cpp
)

ament_target_dependencies(scan_to_costmap_node
  rclcpp
  sensor_msgs
  nav_msgs
  std_msgs
  geometry_msgs
)

# ======================
# Install
# ======================

install(TARGETS
  depth_costmap_node
  scan_to_costmap_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()