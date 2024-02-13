#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "spes_msgs/msg/move_state.hpp"
#include <sensor_msgs/msg/battery_state.hpp>

using namespace BT;

class IsBatteryCharged : public RosTopicSubNode<sensor_msgs::msg::BatteryState>
{
public:
    IsBatteryCharged(const std::string &name,
                     const NodeConfig &conf,
                     const RosNodeParams &params)
        : RosTopicSubNode<sensor_msgs::msg::BatteryState>(name, conf, params)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {InputPort<double>("min_voltage")};
    }

    NodeStatus onTick(const std::shared_ptr<sensor_msgs::msg::BatteryState> &last_msg) override
    {
        double min_voltage;
        getInput<double>("min_voltage", min_voltage);

        if (last_msg == nullptr || last_msg->voltage < min_voltage)
            return NodeStatus::FAILURE;

        return NodeStatus::SUCCESS;
    }
};
