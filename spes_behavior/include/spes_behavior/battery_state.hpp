#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "spes_msgs/msg/move_state.hpp"
#include <sensor_msgs/msg/battery_state.hpp>

using namespace BT;

class BatteryState : public RosTopicSubNode<sensor_msgs::msg::BatteryState>
{
public:
    BatteryState(const std::string &name,
                 const NodeConfig &conf,
                 const RosNodeParams &params)
        : RosTopicSubNode<sensor_msgs::msg::BatteryState>(name, conf, params)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    NodeStatus onTick(const std::shared_ptr<sensor_msgs::msg::BatteryState> &last_msg) override
    {

        if (last_msg == nullptr || last_msg->voltage < 10)
            return NodeStatus::FAILURE;

        return NodeStatus::SUCCESS;
    }
};
