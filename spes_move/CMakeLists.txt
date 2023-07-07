cmake_minimum_required(VERSION 3.8)
project(spes_move)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ruckig REQUIRED)
find_package(spes_msgs REQUIRED)

add_library(
  move
  SHARED
  src/move.cpp
  src/move_node.cpp
)
ament_target_dependencies(
  move
  rclcpp
  nav2_costmap_2d
  pluginlib
  rclcpp_action
  spes_msgs
)
target_include_directories(
  move
  PRIVATE
  include
)
install(
  TARGETS move
  DESTINATION lib
)
target_link_libraries(move ruckig::ruckig)
install(TARGETS move DESTINATION lib/${PROJECT_NAME})

ament_package()
