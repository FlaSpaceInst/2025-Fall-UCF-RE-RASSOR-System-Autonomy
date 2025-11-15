cmake_minimum_required(VERSION 3.5)
project(re_rassor_pathing)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(re_rassor_interfaces REQUIRED)
find_package(re_rassor_cartography REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(
  include
)

add_library(${PROJECT_NAME}_lib
  src/dstarlite.cpp
  src/state.cpp
  src/pathsmoothing.cpp
)

add_library(pathsmooth
  src/pathsmoothing.cpp
)

# Add routing node executable
add_executable(routing_node
  src/routing_node.cpp
)

add_executable(pathsmooth_cli
  src/pathsmooth_main.cpp
)

target_link_libraries(pathsmooth_cli PRIVATE pathsmooth)

# Link routing_node with the library
target_link_libraries(routing_node
  ${PROJECT_NAME}_lib
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  std_msgs
  geometry_msgs
  re_rassor_interfaces
  re_rassor_cartography
)

# Add dependencies for routing_node
ament_target_dependencies(routing_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  re_rassor_interfaces
  re_rassor_cartography
)

target_include_directories(pathsmooth PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  include_directories(
    include
  )

  ament_add_gtest(test_dstar
    tests/test_dstar.cpp
  )
  ament_add_gtest(test_pathsmooth
    tests/test_pathsmooth.cpp
  )
  if(TARGET test_dstar)
    target_link_libraries(test_dstar
      ${PROJECT_NAME}_lib
    )
  endif()
  if(TARGET test_pathsmooth)
    target_link_libraries(test_pathsmooth
      pathsmooth
    )
  endif()
endif()

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS pathsmooth_cli routing_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()