#include <sys/types.h>
#include <sys/stat.h>

#include <filesystem>
#include <iostream>
#include <set>
#include <string>
#include <cstdio>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"

#include "rclcpp/rclcpp.hpp"


int main(int argc, char **argv){
  // Initialize ROS node
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("spes_behavior");


 bool finish = false;
  while (!finish && rclcpp::ok())
  {
    finish = tree.tickOnce() == BT::NodeStatus::SUCCESS;
    tree.sleep(std::chrono::milliseconds(20));

    if (should_exit_on_tree_change && get_last_modification_time() > last_modification_time)
    {
      RCLCPP_WARN(node->get_logger(), "Reloading tree...");
      tree.haltTree();
      tree.sleep(std::chrono::milliseconds(100));
      return 1;
    }
  }
  rclcpp::shutdown();
  return 0;

}