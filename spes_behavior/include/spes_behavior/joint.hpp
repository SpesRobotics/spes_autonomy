#include <string>

#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"


using namespace BT;

class JointAction : public RosTopicPubNode<std_msgs::msg::Float64MultiArray>
{
public:
  JointAction(const std::string& name,
              const NodeConfig& conf,
              const RosNodeParams& params)
    : RosTopicPubNode<std_msgs::msg::Float64MultiArray>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      InputPort<std::string>("data")
    });
  }

  bool setMessage(std_msgs::msg::Float64MultiArray &goal)
  {

    std::string data;
    getInput<std::string>("data", data);
    goal.data.push_back(convertFromString<double>(data));

    std::cout << "JointAction: setGoal" << std::endl;
    for (int i = 0; i < goal.data.size(); i++)
        std::cout << goal.data[i] << std::endl;

    return true;
  }

    // BT::NodeStatus onResultReceived(const WrappedResult &wr) override
    // {
    //     RCLCPP_INFO(node_->get_logger(), "%s: onResultReceived %d", name().c_str(), wr.result->result);
    //     return wr.result->result ? NodeStatus::FAILURE : NodeStatus::SUCCESS;
    // }

    // virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override
    // {
    //     RCLCPP_ERROR(node_->get_logger(), "%s: onFailure %d", name().c_str(), error);
    //     return NodeStatus::FAILURE;
    // }
};




