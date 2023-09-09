#include <string>

#include "behaviortree_ros2/bt_action_node.hpp"
#include "spes_msgs/action/move.hpp"

using namespace BT;

class TranslateAction : public RosActionNode<spes_msgs::action::Move>
{
public:
    TranslateAction(const std::string &name,
                    const NodeConfig &conf,
                    const RosNodeParams &params)
        : RosActionNode<spes_msgs::action::Move>(name, conf, params)
    {
    }

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({
            InputPort<double>("x"),
            InputPort<std::string>("frame_id"),
        });
    }

    bool setGoal(Goal &goal) override {
        double x;
        std::string frame_id;

        getInput<double>("x", goal.target.x);
        getInput<std::string>("frame_id", goal.header.frame_id);

        std::cout << "TranslateAction: setGoal" << std::endl;
        std::cout << "  x: " << x << std::endl;
        std::cout << "  frame_id: " << frame_id << std::endl;

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