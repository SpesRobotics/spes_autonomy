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
            InputPort<uint8_t>("mode"),
            InputPort<double>("tolerance"),
        });
    }

    bool setGoal(Goal &goal) override {
        getInput<uint8_t>("mode", goal.mode);
        getInput<double>("tolerance", goal.tolerance);

        std::cout << "ImageXYawRegulatorAction: setGoal" << std::endl;
        std::cout << "  mode: " << goal.mode << std::endl;
        std::cout << "  tolerance: " << goal.tolerance << std::endl;

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
