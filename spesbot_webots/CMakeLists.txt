cmake_minimum_required(VERSION 3.8)
project(spesbot_webots)


# find dependencies
find_package(ament_cmake REQUIRED)

# resources
install(
  DIRECTORY launch resource data src
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
