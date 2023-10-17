#include <string>

#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace BT;

class JointAction: public RosTopicPubNode<std_msgs::msg::Float64MultiArray>
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
      InputPort<std::string>("joint_names"),
      InputPort<std::string>("positions")
    });
  }

  bool JointAction::setMessage(std_msgs::msg::Float64MultiArray &goal)
  {
    std::string joint_names;
    getInput<std::string>("joint_names", joint_names);
    for (auto &name : splitString(joint_names, ','))
        goal.name.push_back(convertFromString<std::string>(name));

    std::string positions;
    getInput<std::string>("positions", positions);
    for (auto &position : splitString(positions, ','))
        goal.position.push_back(convertFromString<double>(position));

    std::cout << "JointAction: setGoal" << std::endl;
    for (int i = 0; i < goal.name.size(); i++)
        std::cout << "  " << goal.name[i] << ": " << goal.position[i] << std::endl;

    return true;
  }

 BT::NodeStatus onResultReceived(const WrappedResult &wr) override
    {
        RCLCPP_INFO(node_->get_logger(), "%s: onResultReceived %d", name().c_str(), wr.result->error);
        
        setOutput<int>("error", wr.result->error);

        return wr.result->error ? NodeStatus::FAILURE : NodeStatus::SUCCESS;
    }

    virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override
    {
        RCLCPP_ERROR(node_->get_logger(), "%s: onFailure %d", name().c_str(), error);
        return NodeStatus::FAILURE;
    }
};




