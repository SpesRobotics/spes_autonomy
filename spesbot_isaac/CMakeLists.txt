cmake_minimum_required(VERSION 3.8)
project(spesbot_isaac)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/tf2navsatfix.py
  scripts/navsatfix2tf.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  data
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()