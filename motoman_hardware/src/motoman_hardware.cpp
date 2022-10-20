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

    joints_size = info_.joints.size();

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

    // get udp ip address and port
    ip_address = info_.hardware_parameters["udp_ip_address"];
    //udp_port = static_cast<uint16_t>(stoi(info_.hardware_parameters["udp_port"]));
    RCLCPP_INFO(rclcpp::get_logger("MotomanHardware"), "UDP IP %s:%d", ip_address.c_str(), UDP_PORT_REALTIME_MOTION);
    
    // creating udp socket fd
    udp_socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(udp_socket_fd < 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("MotomanHardware"), "udp socket creation failed");
        return hardware_interface::CallbackReturn::FAILURE;
    }

    // set udp server struct
    udp_servaddr.sin_family = AF_INET;
    inet_pton(AF_INET, ip_address.c_str(), &(udp_servaddr.sin_addr.s_addr));
    udp_servaddr.sin_port = htons(UDP_PORT_REALTIME_MOTION);
    sizeof_udp_servaddr = sizeof(udp_servaddr);

    // get tcp ip address and port -> ip same as UDP, port hardcoded

    // creating tcp socket fd
    tcp_socket_fd= socket(AF_INET, SOCK_STREAM, 0);
    if(tcp_socket_fd < 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("MotomanHardware"), "tcp socket creation failed");
        return hardware_interface::CallbackReturn::FAILURE;
    }

    // set tcp client struct
    bzero(&tcp_servaddr, sizeof(tcp_servaddr));
    tcp_servaddr.sin_family = AF_INET;
    inet_pton(AF_INET, ip_address.c_str(), &(tcp_servaddr.sin_addr.s_addr));
    tcp_servaddr.sin_port = htons(TCP_PORT_REALTIME_MOTION);

    // connect the client socket to server socket
    if(connect(tcp_socket_fd, (struct sockaddr*)&tcp_servaddr, sizeof(tcp_servaddr)) != 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("MotomanHardware"), "TCP: Failed to connect to port %d. ERRNO: %d", TCP_PORT_REALTIME_MOTION, errno);
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

    // send tcp motion request #TODO move somewhere else
    simple_message::SimpleMessage motionReq;
    motionReq.prefix.length = sizeof(simple_message::Header) + sizeof(simple_message::SmBodyMotoMotionCtrl);
    motionReq.header.msg_type = simple_message::MsgType::ROS_MSG_MOTO_MOTION_CTRL;
    motionReq.header.comm_type = simple_message::CommType::TOPIC;
    motionReq.header.reply_type = simple_message::ReplyType::INVALID;
    motionReq.body.motionCtrl.command = simple_message::SmCommandType::ROS_CMD_START_RT_MODE;
    
    size_t msg_len = motionReq.prefix.length + sizeof(simple_message::Prefix);
    //RCLCPP_INFO(rclcpp::get_logger("MotomanHardware"), "TCP: sending %ld bytes", msg_len);

    bytesSend = send(tcp_socket_fd, &motionReq, msg_len, 0);
    if(bytesSend <= 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("MotomanHardware"), "Error in TCP MotionReq. ERRNO: %d", errno);
        return CallbackReturn::ERROR;
    }

    // wait for x seconds #TODO better wait for TCP response
    double hw_start_sec_ = 3;
    for (auto i = 0; i < hw_start_sec_; i++)
    {
        RCLCPP_INFO(rclcpp::get_logger("MotomanHardware"), "%.1f seconds left...", hw_start_sec_ - i);
        rclcpp::sleep_for(std::chrono::seconds(1));
    }

    // send controller the UDP client ip
    unsigned char buf[1] = {250}; // #TEST1 run for x sec #TODO protocol
    size_t buf_len = sizeof(buf);
    bytesSend = sendto(udp_socket_fd, buf, buf_len, 0, (const struct sockaddr *) &udp_servaddr, sizeof(udp_servaddr));
   	// #TODO better use select and timeout here, else BLOCKING forever ...
    bytesRecv = recvfrom(udp_socket_fd, buf, buf_len, 0, (struct sockaddr *)&udp_servaddr, &sizeof_udp_servaddr);

    if(buf[0] != 0) // any (not yet received) controller error #TODO protocol 
    {
        RCLCPP_ERROR(rclcpp::get_logger("MotomanHardware"), "Controller not started.");
        return CallbackReturn::FAILURE;
    }

    
    // set default values
    for (auto i = 0u; i < hw_positions_.size(); i++)
    {
        if (std::isnan(hw_positions_[i]))
        {
            hw_positions_[i] = 0;
            hw_velocities_[i] = 0;
            hw_commands_[i] = 0;
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("MotomanHardware"), "Successfully activated!");

    return CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn MotomanHardware::on_deactivate(const rclcpp_lifecycle::State & /* previous_state */)
{
    RCLCPP_INFO(rclcpp::get_logger("MotomanHardware"), "Deactivating ...please wait...");

    // closing sockets
    close(udp_socket_fd);
    close(tcp_socket_fd);

    // #TODO -- make this more useful
    RCLCPP_INFO(rclcpp::get_logger("MotomanHardware"), "Successfully deactivated!");
    return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::return_type MotomanHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    bytesRecv = recvfrom(udp_socket_fd, &state_msg_, sizeof(state_msg_), 0, (struct sockaddr*)&udp_servaddr, &sizeof_udp_servaddr);
    
    if(bytesRecv < 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("MotomanHardware"), "UDP receive error: %d", errno);
        return hardware_interface::return_type::ERROR;
    }

    // set position and velocity to ROS2 control pipeline
    for(uint8_t i = 0; i < joints_size; i++)
    {
        hw_positions_[i] = state_msg_.body.joint_state_ex.joint_state_ex_data[0].pos[i];
        hw_velocities_[i] = state_msg_.body.joint_state_ex.joint_state_ex_data[0].vel[i];
    }
    
    return hardware_interface::return_type::OK;
}


hardware_interface::return_type MotomanHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    // set prefix length
    command_msg_.prefix.length = sizeof(simple_message::Header) + sizeof(simple_message::MotoRealTimeMotionJointCommandEx);

    // set header
    command_msg_.header.msg_type = simple_message::MsgType::MOTO_REALTIME_MOTION_JOINT_COMMAND_EX;
    command_msg_.header.comm_type = simple_message::CommType::TOPIC;
    command_msg_.header.reply_type = simple_message::ReplyType::INVALID;

    // set body
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