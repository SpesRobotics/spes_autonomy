#include <thread>
#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include "spes_move/move.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Executor> executor =
        std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    auto move = std::make_shared<spes_move::Move>("move");

    RCLCPP_INFO(move->get_logger(), "update rate is %d Hz", move->get_update_rate());

    // As here: https://github.com/ros-controls/ros2_control/blob/master/controller_manager/src/ros2_control_node.cpp
    std::thread move_thread(
        [move]()
        {
            auto const period = std::chrono::nanoseconds(1'000'000'000 / move->get_update_rate());
            auto const move_now = std::chrono::nanoseconds(move->now().nanoseconds());
            std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>
                next_iteration_time{move_now};

            rclcpp::Time previous_time = move->now();
            while (rclcpp::ok())
            {
                auto const current_time = move->now();
                previous_time = current_time;

                move->update();

                next_iteration_time += period;
                std::this_thread::sleep_until(next_iteration_time);
            }
        });

    executor->add_node(move);
    executor->spin();
    move_thread.join();
    rclcpp::shutdown();
    return 0;
}