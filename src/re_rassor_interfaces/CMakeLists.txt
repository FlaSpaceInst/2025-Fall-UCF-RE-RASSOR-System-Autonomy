cmake_minimum_required(VERSION 3.8)
project(re_rassor_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}

  "msg/LocationStatus.msg"
  "srv/ObstacleArray.srv"
  "srv/LocationStatus.srv"
  "srv/FinalLocation.srv"
  "msg/ObstacleLocation.msg"
  "msg/ObstacleArray.msg"
  "srv/InitialRoverPosition.srv"
  "srv/InitialObstacleArray.srv"
  "srv/NewPath.srv"
  "msg/YoloMsg.msg"
  "msg/YoloMsgArray.msg"
  "msg/LidarMsg.msg"
  "msg/LidarMsgArray.msg"
  "msg/ObstacleDetectionMsg.msg"
  "msg/ObstacleDetectionMsgArray.msg"
  "srv/NewGoal.srv"
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
