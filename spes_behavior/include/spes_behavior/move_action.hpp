#include <string>

#include "behaviortree_ros2/bt_action_node.hpp"
#include "spes_msgs/action/move.hpp"
#include "spes_msgs/msg/move_command.hpp"

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
            InputPort<bool>("ignore_obstacles"),
            OutputPort<int>("error"),
        });
    }

    bool setGoal(Goal &goal) override {
        getInput<double>("x", goal.target.x);
        getInput<std::string>("frame_id", goal.header.frame_id);
        getInput<bool>("ignore_obstacles", goal.ignore_obstacles);

        std::cout << "TranslateAction: setGoal" << std::endl;
        std::cout << "  x: " << goal.target.x << std::endl;
        std::cout << "  frame_id: " << goal.header.frame_id << std::endl;
        std::cout << "  ignore_obstacles: " << goal.ignore_obstacles << std::endl;

        goal.mode = spes_msgs::msg::MoveCommand::MODE_TRANSLATE;

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


class MoveAction : public RosActionNode<spes_msgs::action::Move>
{
public:
    MoveAction(const std::string &name,
                    const NodeConfig &conf,
                    const RosNodeParams &params)
        : RosActionNode<spes_msgs::action::Move>(name, conf, params)
    {
    }

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({
            InputPort<double>("x"),
            InputPort<double>("y"),
            InputPort<double>("yaw"),
            InputPort<double>("linear_velocity"),
            InputPort<double>("angular_velocity"),
            InputPort<int>("mode"),
            InputPort<std::string>("frame_id"),
            InputPort<bool>("ignore_obstacles"),
            OutputPort<int>("error"),
        });
    }

    bool setGoal(Goal &goal) override {
        int mode;
        double yaw_deg;
        getInput<double>("x", goal.target.x);
        getInput<double>("y", goal.target.y);
        getInput<double>("yaw", yaw_deg);
        goal.target.theta = yaw_deg * M_PI / 180.0;

        getInput<double>("linear_velocity", goal.linear_properties.max_velocity);
        getInput<double>("angular_velocity", goal.angular_properties.max_velocity);
        getInput<int>("mode", mode);
        getInput<std::string>("frame_id", goal.header.frame_id);
        getInput<bool>("ignore_obstacles", goal.ignore_obstacles);

        goal.mode = mode;

        std::cout << "MoveAction: setGoal" << std::endl;
        std::cout << "  x: " << goal.target.x << std::endl;
        std::cout << "  y: " << goal.target.y << std::endl;
        std::cout << "  mode: " << goal.mode << std::endl;
        std::cout << "  frame_id: " << goal.header.frame_id << std::endl;
        std::cout << "  ignore_obstacles: " << goal.ignore_obstacles << std::endl;

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