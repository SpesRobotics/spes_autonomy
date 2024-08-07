#include "spesbot_hardware/serial_hardware_interface.hpp"

#include <vector>
#include <boost/array.hpp>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace spesbot_hardware
{
    const uint8_t START_BYTE = 0x29;
    const uint8_t HEADER_SIZE = 3;
    const uint8_t POSITION_N_BYTES = 2;
    const uint8_t CMD_WRITE_POSITION = 0x01;

    inline void pack_int16(uint8_t *buffer, int16_t value)
    {
        buffer[2] = (value >> 8) & 0xFF;
        buffer[3] = value & 0xFF;
    }

    SerialHardwareInterface::SerialHardwareInterface() : serial_(io_)
    {
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn SerialHardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
        {
            return CallbackReturn::ERROR;
        }

        for (hardware_interface::ComponentInfo component : info.joints)
        {
            Joint joint;
            joint.name = component.name;
            joint.index = std::stoi(component.parameters.at("index"));
            joint.scale = std::stof(component.parameters.at("scale"));
            joint.offset = std::stof(component.parameters.at("offset"));
            joints_.push_back(joint);

            if (joint.index > max_position_command_index_)
                max_position_command_index_ = joint.index;
        }

        return CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn SerialHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
    {
        const std::string port = info_.hardware_parameters["port"];
        const int baudrate_value = std::stod(info_.hardware_parameters["baudrate"]);

        boost::system::error_code error_code;
        serial_.open(port, error_code);
        if (!error_code)
        {
            boost::asio::serial_port_base::baud_rate baudrate_option(baudrate_value);
            serial_.set_option(baudrate_option);

            // Sleep for some time, so the Arduino has time to initialize
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        else
        {
            std::cerr << "ERROR: Cannot connect to " << port << " with error code " << error_code << "\n";
            return CallbackReturn::ERROR;
        }

        return CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn SerialHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
    {
        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> SerialHardwareInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> interfaces;
        return interfaces;
    }

    std::vector<hardware_interface::CommandInterface>
    SerialHardwareInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> interfaces;
        for (Joint &joint : joints_)
        {
            interfaces.emplace_back(hardware_interface::CommandInterface(joint.name, hardware_interface::HW_IF_POSITION, &joint.position_command));
        }
        return interfaces;
    }

    hardware_interface::return_type SerialHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type SerialHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        const int message_length = HEADER_SIZE + (max_position_command_index_ + 1) * POSITION_N_BYTES;
        uint8_t position_command_buffer[message_length];
        
        position_command_buffer[0] = START_BYTE;
        position_command_buffer[1] = message_length;
        position_command_buffer[2] = CMD_WRITE_POSITION;

        for (Joint &joint : joints_)
        {
            int16_t position = joint.position_command * joint.scale + joint.offset;
            pack_int16(&position_command_buffer[HEADER_SIZE + joint.index * POSITION_N_BYTES], position);
            boost::asio::write(serial_, boost::asio::buffer(position_command_buffer, message_length));
        }
        return hardware_interface::return_type::OK;
    }
} // namespace spesbot_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(spesbot_hardware::SerialHardwareInterface, hardware_interface::SystemInterface)
