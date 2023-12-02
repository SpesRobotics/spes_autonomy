#include <string>

#include "spes_msgs/msg/move_command.hpp"

using namespace BT;

class MoveStreamAction : public BT::StatefulActionNode
{
public:
  MoveStreamAction(const std::string &instance_name, const BT::NodeConfig &conf, BT::RosNodeParams params)
      : StatefulActionNode(instance_name, conf),
        node_(params.nh)
  {
    publisher_ = node_->create_publisher<spes_msgs::msg::MoveCommand>(params.default_port_value, 10);
  }

  static BT::PortsList providedPorts()
  {
    return {
        InputPort<double>("x"),
        InputPort<std::string>("frame_id"),
        InputPort<bool>("ignore_obstacles"),
        InputPort<double>("publish_frequency"),
        OutputPort<int>("error"),
    };
  }

  BT::NodeStatus onStart() override
  {
    last_publish_time_ = node_->get_clock()->now();

    getInput<double>("x", message_.target.x);
    getInput<std::string>("frame_id", message_.header.frame_id);
    getInput<bool>("ignore_obstacles", message_.ignore_obstacles);
    getInput<double>("publish_frequency", publish_frequency_);
    message_.mode = spes_msgs::msg::MoveCommand::MODE_TRANSLATE;

    std::cout << "MoveStreamAction: setGoal" << std::endl;
    std::cout << "  x: " << message_.target.x << std::endl;
    std::cout << "  frame_id: " << message_.header.frame_id << std::endl;
    std::cout << "  ignore_obstacles: " << message_.ignore_obstacles << std::endl;
    std::cout << "  publish_frequency: " << publish_frequency_ << std::endl;

    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    if ((node_->get_clock()->now() - last_publish_time_).seconds() >= 1.0 / publish_frequency_)
    {
      publisher_->publish(message_);
      last_publish_time_ = node_->get_clock()->now();
    }
    rclcpp::spin_some(node_);
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    // TODO: This should stop in future
    message_.target.x = 0;
    message_.target.y = 0;
    message_.target.theta = 0;
    publisher_->publish(message_);
  }

private:
  rclcpp::Time last_publish_time_;
  rclcpp::Publisher<spes_msgs::msg::MoveCommand>::SharedPtr publisher_;
  spes_msgs::msg::MoveCommand message_;
  double publish_frequency_;
  std::shared_ptr<rclcpp::Node> node_;
};
