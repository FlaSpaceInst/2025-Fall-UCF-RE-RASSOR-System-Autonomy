cmake_minimum_required(VERSION 3.8)
project(re_rassor_cartography)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Include directories
include_directories(include)

# Install header files
install(
  DIRECTORY include/
  DESTINATION include
)

# Build examples (optional)
option(BUILD_EXAMPLES "Build example programs" OFF)
if(BUILD_EXAMPLES)
  add_executable(obstacle_map_example examples/obstacle_map_example.cpp)
  target_include_directories(obstacle_map_example PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_compile_features(obstacle_map_example PUBLIC cxx_std_17)

  add_executable(simple_test examples/simple_test.cpp)
  target_include_directories(simple_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_compile_features(simple_test PUBLIC cxx_std_17)

  install(TARGETS
    obstacle_map_example
    simple_test
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # GoogleTest
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(obstacle_map_tests test/obstacle_map_tests.cpp)
  target_include_directories(obstacle_map_tests PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_compile_features(obstacle_map_tests PUBLIC cxx_std_17)
endif()

# Export include directories
ament_export_include_directories(include)

ament_package()
