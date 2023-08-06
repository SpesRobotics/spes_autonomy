cmake_minimum_required(VERSION 3.8)
project(spesbot_hardware)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(Boost REQUIRED)

# Serial Hardware Interface
add_library(
  serial_hardware_interface
  SHARED
  src/serial_hardware_interface.cpp
)
target_include_directories(
  serial_hardware_interface
  PRIVATE
  include
  ${Boost_INCLUDE_DIRS}
)
ament_target_dependencies(
  serial_hardware_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
target_link_libraries(serial_hardware_interface ${Boost_LIBRARIES})
pluginlib_export_plugin_description_file(hardware_interface serial_hardware_interface.xml)
install(
  TARGETS serial_hardware_interface
  DESTINATION lib
)

# resources
install(
  DIRECTORY launch resource
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
