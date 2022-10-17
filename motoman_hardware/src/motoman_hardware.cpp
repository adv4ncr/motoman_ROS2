#include "motoman_hardware/motoman_hardware.hpp"


namespace motoman_hardware
{
hardware_interface::CallbackReturn MotomanHardware::on_init(const hardware_interface::HardwareInfo &info)
{
    if ( hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    // check configuration
    for (const hardware_interface::ComponentInfo & joint : info_.joints)
    {
        // check joint command interface size
        if (joint.command_interfaces.size() != 1)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("MotomanHardware"),
                "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        // check joint command interface
        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("MotomanHardware"),
                "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
                joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
        }

        // check joint state interface size
        if (joint.state_interfaces.size() != 2)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("MotomanHardware"),
                "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
                joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        // check first joint state interface
        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("MotomanHardware"),
                "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
                joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        // check second joint state interface
        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("MotomanHardware"),
                "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
                joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    // get ip address and port
    udp_ip_address = info_.hardware_parameters["udp_ip_address"];
    udp_port = static_cast<uint16_t>(stoi(info_.hardware_parameters["udp_port"]));
    RCLCPP_INFO(rclcpp::get_logger("MotomanHardware"), "UDP IP %s:%d", udp_ip_address.c_str(), udp_port);

    // creating socket fd
    udp_socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(udp_socket_fd < 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("MotomanHardware"), "socket creation failed");
        return hardware_interface::CallbackReturn::FAILURE;
    }

    // set server struct
    udp_servaddr.sin_family = AF_INET;
    udp_servaddr.sin_addr.s_addr = inet_addr(udp_ip_address.c_str());
    udp_servaddr.sin_port = htons(udp_port);
    sizeof_udp_servaddr = sizeof(udp_servaddr);

    // bind to port
    if(bind(udp_socket_fd, (struct sockaddr*)&udp_servaddr, sizeof(udp_servaddr)) < 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("MotomanHardware"), "Failed to bind to port %d", udp_port);
        return hardware_interface::CallbackReturn::FAILURE;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> MotomanHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (auto i = 0u; i < info_.joints.size(); i++)
    {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    }

    return state_interfaces;
}


std::vector<hardware_interface::CommandInterface> MotomanHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (auto i = 0u; i < info_.joints.size(); i++)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
    }

    return command_interfaces;
}


hardware_interface::CallbackReturn MotomanHardware::on_activate(const rclcpp_lifecycle::State & /* previous_state */)
{
    RCLCPP_INFO(rclcpp::get_logger("MotomanHardware"), "Starting ...please wait...");

    // #TODO -- make this more useful
    
    // set some default values
    for (auto i = 0u; i < hw_positions_.size(); i++)
    {
        if (std::isnan(hw_positions_[i]))
        {
            hw_positions_[i] = 0;
            hw_velocities_[i] = 0;
            hw_commands_[i] = 0;
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully activated!");

    return CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn MotomanHardware::on_deactivate(const rclcpp_lifecycle::State & /* previous_state */)
{
    RCLCPP_INFO(rclcpp::get_logger("MotomanHardware"), "Deactivating ...please wait...");
    // #TODO -- make this more useful
    RCLCPP_INFO(rclcpp::get_logger("MotomanHardware"), "Successfully deactivated!");
    return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::return_type MotomanHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    bytesReceived = recvfrom(udp_socket_fd, &state_msg_, sizeof(state_msg_), 0, (struct sockaddr*)&udp_servaddr, &sizeof_udp_servaddr);

    if(bytesReceived < 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("MotomanHardware"), "UDP receive error: %d", errno);
        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
}


hardware_interface::return_type MotomanHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    command_msg_.header.msg_type = simple_message::MsgType::MOTO_REALTIME_MOTION_JOINT_COMMAND_EX;
    command_msg_.header.comm_type = simple_message::CommType::TOPIC;
    command_msg_.header.reply_type = simple_message::ReplyType::INVALID;

    command_msg_.body.joint_command_ex.message_id = state_msg_.body.joint_state_ex.message_id;
    command_msg_.body.joint_command_ex.number_of_valid_groups = state_msg_.body.joint_state_ex.number_of_valid_groups;

    if(sendto(udp_socket_fd, &command_msg_, sizeof(command_msg_), MSG_CONFIRM, (const struct sockaddr *) &udp_servaddr, sizeof(udp_servaddr)) <0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("MotomanHardware"), "UDP send error: %d", errno);
        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
}


} // namespace motoman_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(motoman_hardware::MotomanHardware, hardware_interface::SystemInterface)