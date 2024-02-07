#ifndef SPESBOT_HARDWARE__SERIAL_HARDWARE_INTERFACE_HPP_
#define SPESBOT_HARDWARE__SERIAL_HARDWARE_INTERFACE_HPP_

#include <vector>
#include <boost/asio.hpp>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

namespace spesbot_hardware
{
    class Joint
    {
    public:
        double position_command;
        std::string name;
        uint8_t index;
        double scale;
        double offset;
    };

    class SerialHardwareInterface : public hardware_interface::SystemInterface
    {
    public:
        SerialHardwareInterface();
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
        hardware_interface::return_type read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;
        hardware_interface::return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

    private:
        std::vector<Joint> joints_;
        boost::asio::io_service io_;
        boost::asio::serial_port serial_;

        uint8_t max_position_command_index_{0};
    };
} // namespace spesbot_hardware

#endif // SPESBOT_HARDWARE__SERIAL_HARDWARE_INTERFACE_HPP_