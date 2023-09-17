cmake_minimum_required(VERSION 3.8)
project(spes_tests)

find_package(ament_cmake REQUIRED)

# Launch
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Resources
install(DIRECTORY
  resource
  DESTINATION share/${PROJECT_NAME}
)

# Scripts
install(PROGRAMS
  scripts/test_move_latency_compensation
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
