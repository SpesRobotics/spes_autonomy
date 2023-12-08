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
            InputPort<int>("direction"),
            InputPort<double>("tolerance"),
            InputPort<double>("image_segment"),
            InputPort<double>("kp"),
            InputPort<int>("object_class"),
            InputPort<int>("timeout"),
            InputPort<std::string>("type"),
        });
    }

    bool setGoal(Goal &goal) override
    {
        int direction;
        int object_class;
        int timeout;

        getInput<int>("direction", direction);
        goal.direction = (uint8_t)direction;
        getInput<int>("object_class", object_class);
        goal.object_class = (uint8_t)object_class;
        getInput<int>("timeout", timeout);
        goal.timeout = (uint8_t)timeout;

        getInput<double>("tolerance", goal.tolerance);
        getInput<double>("image_segment", goal.image_segment);
        getInput<double>("kp", goal.kp);
        getInput<std::string>("type", goal.type);

        std::cout << "ImageXYawRegulatorAction: setGoal" << std::endl;
        std::cout << "  direction: " << direction << std::endl;
        std::cout << "  tolerance: " << goal.tolerance << std::endl;
        std::cout << "  type: " << goal.type << std::endl;
        std::cout << "  image_segment: " << goal.image_segment << std::endl;
        std::cout << "  kp: " << goal.kp << std::endl;
        std::cout << "  object_class: " << object_class << std::endl;
        std::cout << "  timeout: " << timeout << std::endl;

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
