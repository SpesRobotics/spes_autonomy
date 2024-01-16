cmake_minimum_required(VERSION 3.8)
project(spes_behavior)

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(spes_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

get_filename_component(BEHAVIOR_DIRECTORY behaviors ABSOLUTE [CACHE])
add_compile_definitions(BEHAVIOR_DIRECTORY="${BEHAVIOR_DIRECTORY}")

# Behavior
add_executable(behavior src/behavior.cpp)
ament_target_dependencies(
  behavior
  rclcpp
  rclcpp_action
  spes_msgs
  sensor_msgs
  behaviortree_cpp
  behaviortree_ros2
)
target_include_directories(behavior PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS behavior
  DESTINATION lib/${PROJECT_NAME})
install(
  DIRECTORY include/
  DESTINATION include
)

# Package
ament_export_dependencies(
    rclcpp
    rclcpp_action
    spes_msgs
    sensor_msgs
    behaviortree_cpp
    behaviortree_ros2
)
ament_export_include_directories(
  include
)
ament_package()
