#include <string>

#include "behaviortree_ros2/bt_action_node.hpp"
#include "spes_msgs/action/image_x_yaw_regulate.hpp"

using namespace BT;

class ImageXYawRegulatorAction : public RosActionNode<spes_msgs::action::ImageXYawRegulate>
{
public:
    ImageXYawRegulatorAction(const std::string &name,
                             const NodeConfig &conf,
                             const RosNodeParams &params)
        : RosActionNode<spes_msgs::action::ImageXYawRegulate>(name, conf, params)
    {
    }

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({
            InputPort<int>("mode"),
            InputPort<double>("tolerance"),
            InputPort<double>("overshoot_value"),
            InputPort<double>("image_segment"),
            InputPort<std::string>("overshoot_type"),
        });
    }

    bool setGoal(Goal &goal) override
    {
        int mode;

        getInput<int>("mode", mode);
        goal.mode = (uint8_t)mode;
        getInput<double>("tolerance", goal.tolerance);
        getInput<double>("overshoot_value", goal.overshoot_value);
        getInput<double>("image_segment", goal.image_segment);
        getInput<std::string>("overshoot_type", goal.overshoot_type);

        std::cout << "ImageXYawRegulatorAction: setGoal" << std::endl;
        std::cout << "  mode: " << std::to_string(goal.mode) << std::endl;
        std::cout << "  tolerance: " << goal.tolerance << std::endl;
        std::cout << "  overshoot_value: " << goal.overshoot_value << std::endl;
        std::cout << "  overshoot_type: " << goal.overshoot_type << std::endl;
        std::cout << "  image_segment: " << goal.image_segment << std::endl;

        return true;
    }

    BT::NodeStatus onResultReceived(const WrappedResult &wr) override
    {
        RCLCPP_INFO(node_->get_logger(), "%s: onResultReceived %d", name().c_str(), wr.result->result);
        return wr.result->result ? NodeStatus::FAILURE : NodeStatus::SUCCESS;
    }

    virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override
    {
        RCLCPP_ERROR(node_->get_logger(), "%s: onFailure %d", name().c_str(), error);
        return NodeStatus::FAILURE;
    }
};
