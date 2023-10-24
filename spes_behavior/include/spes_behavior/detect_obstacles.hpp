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

    if(!last_msg || last_msg->error == spes_msgs::msg::MoveState::ERROR_OBSTACLE)
    {
      RCLCPP_INFO(logger(), "New message: AKCIJA JE PUKLA"); 
      return NodeStatus::FAILURE;  
    }
    return NodeStatus::SUCCESS;
  }
};