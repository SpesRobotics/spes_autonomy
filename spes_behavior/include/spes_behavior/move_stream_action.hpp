#include <string>

#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "spes_msgs/msg/move_command.hpp"


using namespace BT;

class MoveStreamAction : public RosTopicPubNode<spes_msgs::msg::MoveCommand>
{
public:
  MoveStream(const std::string& name,
              const NodeConfig& conf,
              const RosNodeParams& params)
    : RosTopicPubNode<spes_msgs::msg::MoveCommand>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
            InputPort<double>("x"),
            InputPort<std::string>("frame_id"),
            InputPort<bool>("ignore_obstacles"),
            OutputPort<int>("error"),
        });
  }

  bool setMessage(spes_msgs::msg::MoveCommand &goal)
  {
    getInput<double>("x", goal.target.x);
    getInput<std::string>("frame_id", goal.header.frame_id);
    getInput<bool>("ignore_obstacles", goal.ignore_obstacles);

    std::cout << "MoveStreamAction: setGoal" << std::endl;
    std::cout << "  x: " << goal.target.x << std::endl;
    std::cout << "  frame_id: " << goal.header.frame_id << std::endl;
    std::cout << "  ignore_obstacles: " << goal.ignore_obstacles << std::endl;

    goal.mode = spes_msgs::msg::MoveCommand::MODE_TRANSLATE;

    return true;
  }
};




