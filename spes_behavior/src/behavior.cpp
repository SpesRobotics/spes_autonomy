#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "rclcpp/rclcpp.hpp"

#include "behaviortree_ros2/plugins.hpp"
#include "spes_behavior/move_action.hpp"
#include "spes_behavior/image_x_yaw_regulator_action.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("spes_behavior");
    BT::Blackboard::Ptr blackboard = BT::Blackboard::create();

    node->declare_parameter<std::string>("behavior", "");
    std::string behavior = node->get_parameter("behavior").as_string();

    BT::BehaviorTreeFactory factory;
    BT::RosNodeParams params;
    params.nh = node;

    params.default_port_value = "move/move";
    factory.registerNodeType<TranslateAction>("Translate", params);

    params.default_port_value = "image_x_yaw_regulator/regulate";
    factory.registerNodeType<ImageXYawRegulatorAction>("ImageXYawRegulator", params);

    using std::filesystem::directory_iterator;
    for (auto const &entry : directory_iterator(BEHAVIOR_DIRECTORY))
        if (entry.path().extension() == ".xml")
            factory.registerBehaviorTreeFromFile(entry.path().string());
    BT::Tree tree = factory.createTree(behavior, blackboard);
    BT::StdCoutLogger logger_cout(tree);

    bool finish = false;
    while (!finish && rclcpp::ok())
    {
        finish = tree.tickOnce() == BT::NodeStatus::SUCCESS;
        tree.sleep(std::chrono::milliseconds(10));
    }
    rclcpp::shutdown();
    return 0;
}
