cmake_minimum_required(VERSION 3.8)
project(re_rassor_test)

# ── Dependencies ─────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)

# ── Install hw_command_relay script ──────────────────────────────────────────
# This is a standalone executable script, NOT a Python package — so we use
# install(PROGRAMS) directly.  No ament_python_install_package() needed and
# no re_rassor_test/__init__.py required.
install(PROGRAMS
  scripts/hw_command_relay.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME hw_command_relay
)

install(PROGRAMS
  scripts/synthetic_camera_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME synthetic_camera_publisher
)

# ── Install launch / config / world / model files ────────────────────────────
install(DIRECTORY
  launch
  config
  worlds
  models
  DESTINATION share/${PROJECT_NAME}
)

# ── Testing ───────────────────────────────────────────────────────────────────
if(BUILD_TESTING)

  # ── C++ unit tests ──────────────────────────────────────────────────────
  find_package(ament_cmake_gtest REQUIRED)  # scoped inside BUILD_TESTING

  ament_add_gtest(test_depth_math
    test/unit/test_depth_math.cpp
  )
  target_include_directories(test_depth_math PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  # ── Python / launch_testing integration test ────────────────────────────
  # This is the critical block that makes
  #   ros2 launch re_rassor_test e2e_obstacle_detection.launch.py
  # also run the pytest suite via `colcon test`.
  #
  # launch_testing_ament_cmake provides add_launch_test() which:
  #   1. Calls generate_test_description() from the launch file
  #   2. Waits for ReadyToTest() to fire
  #   3. Runs all pytest-marked test cases in the associated test file
  #   4. Tears down the launch after the tests finish
  find_package(launch_testing_ament_cmake REQUIRED)

  add_launch_test(
    launch/e2e_obstacle_detection.launch.py        # launch file
    TARGET e2e_obstacle_detection                  # CTest target name
    TIMEOUT 120                                    # seconds — Nav2 needs time
    ENV
      # Suppress gz sim GUI on CI
      GZ_HEADLESS_RENDERING=1
  )

  # Tell launch_testing where the pytest test file lives.
  # launch_testing will import this file after ReadyToTest() fires.
  set_tests_properties(e2e_obstacle_detection PROPERTIES
    ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test/e2e:$ENV{PYTHONPATH}"
    LABELS "e2e"
  )

  # ── Integration test (node-level, no Gazebo) ─────────────────────────────
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(test_obstacle_detection_node
    test/integration/test_obstacle_detection_node.py
    TIMEOUT 120
    ENV
      RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  )

endif()

ament_package()