cmake_minimum_required(VERSION 3.8)
project(spesbot_webots)


# find dependencies
find_package(ament_cmake REQUIRED)

# resources
install(
  DIRECTORY launch resource data
  DESTINATION share/${PROJECT_NAME}
)

# Python scripts
install(PROGRAMS
  scripts/ros2virtualcam
  scripts/visual_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
