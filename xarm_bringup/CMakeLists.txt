cmake_minimum_required(VERSION 3.8)
project(xarm_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf launch resource
  DESTINATION share/${PROJECT_NAME}
)

# install scripts
install(
  PROGRAMS scripts/sixd_speed_limiter
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
