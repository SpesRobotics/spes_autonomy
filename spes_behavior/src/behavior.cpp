#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "rclcpp/rclcpp.hpp"

#include "behaviortree_ros2/plugins.hpp"
#include "spes_behavior/move_action.hpp"
#include "spes_behavior/image_x_yaw_regulator_action.hpp"
#include "spes_behavior/joint.hpp"
#include "spes_behavior/move_stream_action.hpp"
#include "spes_behavior/is_path_clear.hpp"
#include "spes_behavior/is_object_detected.hpp"
#include "spes_behavior/battery_state.hpp"

#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <iostream>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <thread>

using tcp = boost::asio::ip::tcp;
namespace http = boost::beast::http;

void handleRequest(http::request<http::string_body> &request, tcp::socket &socket, BT::Blackboard::Ptr blackboard)
{
    try
    {
        if (request.method() == http::verb::get && request.target() == "/rotate")
        {

            http::response<http::string_body> response;
            response.version(request.version());
            response.result(http::status::ok);
            response.set(http::field::server, "My HTTP Server");
            response.set(http::field::content_type, "text/html");

            blackboard->set("rotate", true);
            response.body() = "<p>Request accepted!</p>";

            response.prepare_payload();
            boost::beast::http::write(socket, response);
        }
        else
        {
            std::cout << "Not Found - Method: " << request.method_string().to_string()
                      << ", Target: " << request.target().to_string() << std::endl;

            http::response<http::string_body> response;
            response.version(request.version());
            response.result(http::status::not_found);
            response.set(http::field::server, "My HTTP Server");
            response.set(http::field::content_type, "text/html");
            response.body() = "Not found";

            response.prepare_payload();
            boost::beast::http::write(socket, response);
        }
    }
    catch (const boost::wrapexcept<boost::system::system_error> &ex)
    {
        std::cerr << "Boost System Error: " << ex.what() << std::endl;
    }
    catch (const std::exception &ex)
    {
        std::cerr << "Exception: " << ex.what() << std::endl;
    }
    catch (...)
    {
        std::cerr << "Unknown exception occurred." << std::endl;
    }
}

void runServer(BT::Blackboard::Ptr blackboard)
{
    boost::asio::io_context io_context;
    tcp::acceptor acceptor(io_context, {tcp::v4(), 8081});
    std::cout << "Server started!" << std::endl;
    while (true)
    {
        tcp::socket socket(io_context);
        acceptor.accept(socket);
        boost::beast::flat_buffer buffer;
        http::request<http::string_body> request;
        boost::beast::http::read(socket, buffer, request);

        handleRequest(request, socket, blackboard);
        socket.shutdown(tcp::socket::shutdown_send);
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("spes_behavior");
    BT::Blackboard::Ptr blackboard = BT::Blackboard::create();
    blackboard->set("rotate", false);

    node->declare_parameter<std::string>("behavior", "");
    std::string behavior = node->get_parameter("behavior").as_string();

    BT::BehaviorTreeFactory factory;
    BT::RosNodeParams params;
    params.nh = node;

    std::thread serverThread(runServer, blackboard);

    params.default_port_value = "move/move";
    factory.registerNodeType<TranslateAction>("Translate", params);
    factory.registerNodeType<MoveAction>("Move", params);

    params.default_port_value = "move/command";
    factory.registerNodeType<MoveStreamAction>("MoveStream", params);

    params.default_port_value = "move/state";
    factory.registerNodeType<IsPathClear>("IsPathClear", params);

    params.default_port_value = "have_detection";
    factory.registerNodeType<IsObjectDetected>("IsObjectDetected", params);

    params.default_port_value = "image_x_yaw_regulator/regulate";
    factory.registerNodeType<ImageXYawRegulatorAction>("ImageXYawRegulator", params);

    params.default_port_value = "removal_velocity_controller/commands";
    factory.registerNodeType<JointAction>("Joint", params);

    params.default_port_value = "battery_state";
    factory.registerNodeType<BatteryState>("isFullyBattery", params);

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
    serverThread.join();
    rclcpp::shutdown();
    return 0;
}
