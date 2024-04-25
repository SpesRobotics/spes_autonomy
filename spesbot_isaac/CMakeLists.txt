cmake_minimum_required(VERSION 3.8)
project(spesbot_isaac)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  data
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()