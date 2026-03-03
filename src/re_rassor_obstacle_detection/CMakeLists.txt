cmake_minimum_required(VERSION 3.8)
project(re_rassor_obstacle_detection)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(CURL REQUIRED)

add_executable(depth_costmap_node src/depth_costmap_node.cpp)

ament_target_dependencies(depth_costmap_node
  rclcpp
  nav_msgs
  std_msgs
)

target_link_libraries(depth_costmap_node
  CURL::libcurl
)

install(TARGETS
  depth_costmap_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()