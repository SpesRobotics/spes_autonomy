#include "bt_topic_sub_node.hpp"
#include "spes_msgs/msg/move_state.hpp"

using namespace BT;

class DetectObstacles: public RosTopicSubNode<spes_msgs::msg::MoveState>
{
public:
  DetectObstacles(const std::string& name,
                const NodeConfig& conf,
                const RosNodeParams& params)
    : RosTopicSubNode<spes_msgs::msg::MoveState>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return {};
  }

  NodeStatus onTick(const std::shared_ptr<spes_msgs::msg::MoveState>& last_msg) override
  {
    RCLCPP_INFO(logger(), "New message: AKCIJA JE PUKLA %d", last_msg->state); 
    if(last_msg /*->state == spes_msgs::msg::MoveState::ERROR_OBSTACLE*/) // empty if no new message received, since the last tick
    {
      RCLCPP_INFO(logger(), "New message: AKCIJA JE PUKLA"); 
      return NodeStatus::FAILURE;  
    }
    return NodeStatus::SUCCESS;
  }
};