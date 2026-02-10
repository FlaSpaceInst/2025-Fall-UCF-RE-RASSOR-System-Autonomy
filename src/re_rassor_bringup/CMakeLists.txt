cmake_minimum_required(VERSION 3.8)
project(re_rassor_bringup)

find_package(ament_cmake REQUIRED)

# install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# install config, if needed 
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# install maps, if needed
install(DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
