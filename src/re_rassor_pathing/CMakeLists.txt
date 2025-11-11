cmake_minimum_required(VERSION 3.5)
project(re_rassor_pathing)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(re_rassor_interfaces REQUIRED)
find_package(re_rassor_cartography REQUIRED)


include_directories(
  include
)

add_library(${PROJECT_NAME}_lib
  src/dstarlite.cpp
  src/state.cpp
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  std_msgs
  re_rassor_interfaces
  re_rassor_cartography
)


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  include_directories(
    include
  )

  ament_add_gtest(test_dstar
    tests/test_dstar.cpp
  )

  if(TARGET test_dstar)
    target_link_libraries(test_dstar
      ${PROJECT_NAME}_lib
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

ament_package()
