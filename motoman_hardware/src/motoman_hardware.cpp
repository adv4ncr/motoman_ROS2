#include "motoman_hardware/motoman_hardware.hpp"
#include "motoman_hardware/simple_message.hpp"
#include "motoman_hardware/status_server.hpp"
#include "motoman_hardware/udp_rt_protocol.h"
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <mutex>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <unistd.h>

namespace motoman_hardware
{

// Default constructor
MotomanHardware::MotomanHardware() :
    logger(rclcpp::get_logger("motoman_hardware"))
{
    // Bind shutdown function, waiting for proper fix. See https://github.com/ros-controls/ros2_control/issues/472 
    rclcpp::on_shutdown(std::bind(&MotomanHardware::shutdown_helper, this));
}

void MotomanHardware::shutdown_helper()
{
    RCLCPP_WARN(logger, "shutdown helper");
    MotomanHardware::on_shutdown(this->get_state());
}

hardware_interface::CallbackReturn MotomanHardware::on_init(const hardware_interface::HardwareInfo &info)
{
    if ( hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // ----------- Initialize state publisher -----------

    // eprosima::fastdds::dds::DomainParticipantQos _participantQos;
    // _participantQos.name("Participant_publisher");
    // eprosima::fastdds::dds::DomainId_t _domain_ID = 12;
    // RCLCPP_WARN(logger, "[DDS] setting domain ID: %d", _domain_ID);
    // _state_domain_participant = eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->create_participant(_domain_ID, _participantQos);
    // if (!_state_domain_participant)
    // {
    //     RCLCPP_ERROR(logger, "[DDS] failed to set participant");
    //     return hardware_interface::CallbackReturn::ERROR;
    // }
    
    // // Register the type
    // _state_type = eprosima::fastdds::dds::TypeSupport(new motoman_description::msg::RobotStatePubSubType());
    // _state_type.register_type(_state_domain_participant);
    
    // // Create the Publisher
    // _state_publisher = _state_domain_participant->create_publisher(eprosima::fastdds::dds::PUBLISHER_QOS_DEFAULT, nullptr);
    // if (!_state_publisher)
    // {
    //     RCLCPP_ERROR(logger, "[DDS] failed to set publisher");
    //     return hardware_interface::CallbackReturn::ERROR;
    // }
    
    // // Create the publications Topic
    // _state_topic = _state_domain_participant->create_topic("rt/robot_controller_state", _state_type.get_type_name(), eprosima::fastdds::dds::TOPIC_QOS_DEFAULT);
    // if (!_state_topic)
    // {
    //     RCLCPP_ERROR(logger, "[DDS] failed to set topic");
    //     return hardware_interface::CallbackReturn::ERROR;
    // }

    // // Create the DataWriter
    // state_data_writer = _state_publisher->create_datawriter(_state_topic, eprosima::fastdds::dds::DATAWRITER_QOS_DEFAULT, nullptr);
    // if (!state_data_writer)
    // {
    //     RCLCPP_ERROR(logger, "[DDS] failed to set DataWriter");
    //     return hardware_interface::CallbackReturn::ERROR;
    // }

    // ----------- Initialize hardware interface -----------
    
    // Set current joint size
    joints_size = info_.joints.size();
    
    hw_commands.resize(joints_size, std::numeric_limits<double>::quiet_NaN());
    hw_cmd_initial.resize(joints_size, std::numeric_limits<double>::quiet_NaN());
    hw_cmd_prev.resize(joints_size, std::numeric_limits<double>::quiet_NaN());
    hw_pos_snd.resize(joints_size, std::numeric_limits<double>::quiet_NaN());
    hw_pos_cmd.resize(joints_size, std::numeric_limits<double>::quiet_NaN());
    hw_pos_set.resize(joints_size, std::numeric_limits<double>::quiet_NaN());
    hw_pos_fb.resize(joints_size, std::numeric_limits<double>::quiet_NaN());
    hw_vel_cmd.resize(joints_size, std::numeric_limits<double>::quiet_NaN());
    hw_vel_set.resize(joints_size, std::numeric_limits<double>::quiet_NaN());
    hw_vel_fb.resize(joints_size, std::numeric_limits<double>::quiet_NaN());
    hw_acc_cmd.resize(joints_size, std::numeric_limits<double>::quiet_NaN());
    hw_acc_set.resize(joints_size, std::numeric_limits<double>::quiet_NaN());
    //hw_state_msg.resize(joints_size, std::numeric_limits<double>::quiet_NaN());

    // Validate the udp_rt_protocol parameters
    if(joints_size > RT_ROBOT_JOINTS_MAX)
    {
        RCLCPP_ERROR(logger, 
            "Configured joint size (%ld) > RT_ROBOT_JOINTS_MAX (%d). Exit.", joints_size,RT_ROBOT_JOINTS_MAX);
        return hardware_interface::CallbackReturn::ERROR;
    }



    // check configuration
    for (const hardware_interface::ComponentInfo & joint : info_.joints) {
        // check joint command interface size
        if (joint.command_interfaces.size() != 1)
        {
            RCLCPP_FATAL(
                logger,
                "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        // check joint command interface
        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
            RCLCPP_FATAL(
                logger,
                "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
                joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        // // check joint state interface size
        // if (joint.state_interfaces.size() != 2)
        // {
        //     RCLCPP_FATAL(
        //         logger,
        //         "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        //         joint.state_interfaces.size());
        //     return hardware_interface::CallbackReturn::ERROR;
        // }

        // check first joint state interface
        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
            RCLCPP_FATAL(
                logger,
                "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
                joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        // check second joint state interface
        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
        {
            RCLCPP_FATAL(
                logger,
                "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
                joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
        }

        // There are actually 2 more.
        // 2 position states (set, feedback)
        // 2 velocity states (set, feedback)
    }

    // get udp ip address and port
    ip_address = info_.hardware_parameters["udp_ip_address"];
    //udp_port = static_cast<uint16_t>(stoi(info_.hardware_parameters["udp_port"]));
    RCLCPP_INFO(logger, "UDP IP %s:%d", ip_address.c_str(), REALTIME_MOTION_UDP_PORT);
    
    // creating udp socket fd
    udp_socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(udp_socket_fd < 0)
    {
        RCLCPP_ERROR(logger, "udp socket creation failed");
        return hardware_interface::CallbackReturn::FAILURE;
    }

    // Set file descriptor
    pfds.fd = udp_socket_fd;
    pfds.events = POLL_IN;


    // set udp server struct
    udp_servaddr.sin_family = AF_INET;
    inet_pton(AF_INET, ip_address.c_str(), &(udp_servaddr.sin_addr.s_addr));
    udp_servaddr.sin_port = htons(REALTIME_MOTION_UDP_PORT);
    sizeof_udp_servaddr = sizeof(udp_servaddr);

    // get tcp ip address and port -> ip same as UDP, port hardcoded

    // creating tcp socket fd
    tcp_socket_fd= socket(AF_INET, SOCK_STREAM, 0);
    if(tcp_socket_fd < 0)
    {
        RCLCPP_ERROR(logger, "tcp socket creation failed");
        return hardware_interface::CallbackReturn::FAILURE;
    }

    // set tcp client struct
    bzero(&tcp_servaddr, sizeof(tcp_servaddr));
    tcp_servaddr.sin_family = AF_INET;
    inet_pton(AF_INET, ip_address.c_str(), &(tcp_servaddr.sin_addr.s_addr));
    tcp_servaddr.sin_port = htons(TCP_PORT_MOTION_COMMAND);

    // set timeout structs
    tcp_timeout.tv_sec = TCP_TIMEOUT_S;
    tcp_timeout.tv_nsec = 0;
    // udp_timeout.tv_sec = 0;
    // udp_timeout.tv_nsec = UDP_TIMEOUT_NS;


    return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn MotomanHardware::on_cleanup(const rclcpp_lifecycle::State & /* previous_state */)
{
    RCLCPP_INFO(logger, "Cleaning up ...");
    
    // closing sockets
    close(udp_socket_fd);
    close(tcp_socket_fd);

    return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn MotomanHardware::on_configure(const rclcpp_lifecycle::State & /* previous_state */)
{
    RCLCPP_INFO(logger, "Configure Hardware ...");


    // connect the client socket to server socket
    if(connect(tcp_socket_fd, (struct sockaddr*)&tcp_servaddr, sizeof(tcp_servaddr)) != 0)
    {
        RCLCPP_ERROR(logger, "TCP: Failed to connect to port %d. ERRNO: %d", TCP_PORT_MOTION_COMMAND, errno);
        return hardware_interface::CallbackReturn::FAILURE;
    }

    
    // "connect" to udp socket - enables recv / send instead of recvfrom / sendto
    if(connect(udp_socket_fd, (struct sockaddr*)&udp_servaddr, sizeof_udp_servaddr) != 0)
    {
        RCLCPP_ERROR(logger, "UDP: Failed to connect to port %d. ERRNO: %d", REALTIME_MOTION_UDP_PORT, errno);
        return hardware_interface::CallbackReturn::FAILURE; 
    }

    // Setup robot status server
    set_robot_status_thread(THREAD_STATE::START);

    // Clear msgs
    memset(&rtMsgSend_, 0, RT_MSG_CMD_SIZE);
    memset(&rtMsgRecv_, 0, RT_MSG_STATE_SIZE);

    // Set current position as first hardware command
    //initial_controller_commands = false;

    RCLCPP_INFO(logger, "Hardware configured.");
    return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn MotomanHardware::on_deactivate(const rclcpp_lifecycle::State & /* previous_state */)
{
    RCLCPP_WARN(logger, "Deactivating ...please wait...");

    if(send_tcp_motion_request(simple_message::SmCommandType::ROS_CMD_STOP_RT_MODE) != REQUEST_RETURN_TYPE::SUCCESS)
    {
        RCLCPP_ERROR(logger, "Error in TCP request. ERRNO: %d", errno);
        //return CallbackReturn::ERROR;
    }

    // Stop robot status server
    set_robot_status_thread(THREAD_STATE::STOP);

    _is_deactivated = true;

    // RCLCPP_WARN(logger, "Successfully deactivated!");
    return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn MotomanHardware::on_activate(const rclcpp_lifecycle::State & /* previous_state */)
{
    RCLCPP_INFO(logger, "Starting ...please wait...");


    // set default values
    robot_controller_initialized = false;

    for (auto i = 0u; i < joints_size; i++)
    {
        if (std::isnan(hw_pos_set[i]))
        {
            hw_commands[i] = 0;
            hw_pos_snd[i] = 0;
            hw_pos_cmd[i] = 0;
            hw_pos_set[i] = 0;
            hw_pos_fb[i] = 0;
            hw_vel_cmd[i] = 0;
            hw_vel_set[i] = 0;
            hw_vel_fb[i] = 0;
            hw_acc_cmd[i] = 0;
            hw_acc_set[i] = 0;
            //hw_state_msg[i] = 0;
        }
    }


    // Request robot controller
    if(send_tcp_motion_request(simple_message::SmCommandType::ROS_CMD_START_RT_MODE) != REQUEST_RETURN_TYPE::SUCCESS) {
        RCLCPP_ERROR(logger, "Error in TCP request. ERRNO: %d", errno);
        return CallbackReturn::FAILURE;
    }

    // set msgs
    udp_rt_message::RtMsg start_msg;
    memset(&start_msg, 0, RT_MSG_CMD_SIZE);
    start_msg.header.msg_state = udp_rt_message::STATE_INITIALIZE;
    start_msg.header.msg_code = udp_rt_message::CODE_UNDEFINED;

    // Send controller the UDP client ip
    bytesSend = send(udp_socket_fd, &start_msg, RT_MSG_CMD_SIZE, 0);

    // clear message
    memset(&rtMsgRecv_, 0, RT_MSG_STATE_SIZE);

    // receive UDP message
    bytesRecv = recv(udp_socket_fd, &rtMsgRecv_, RT_MSG_STATE_SIZE, 0);

    // Initialize robot controller
    if (rtMsgRecv_.header.msg_state == udp_rt_message::STATE_IDLE) {
        // Init commands: (set current robot joit angles as first commands)
        {
            //std::lock_guard<std::mutex> lock(start_mutex);
            for(uint8_t i = 0; i < joints_size; i++)
            {
                hw_commands[i] = rtMsgRecv_.body.state[0].pos_fb[i];
                hw_cmd_initial[i] = rtMsgRecv_.body.state[0].pos_fb[i];
                hw_cmd_prev[i] = rtMsgRecv_.body.state[0].pos_fb[i];
            }
            //start_signal.store(StartSignalIndicator::SUCCESS);
        }
        // Signal activation thread
        //start_condition.notify_one();
        // Set initialization signal
        robot_controller_initialized = true;
        RCLCPP_INFO(logger, "activation done");
        return CallbackReturn::SUCCESS;
    }
    else if (rtMsgRecv_.header.msg_state == udp_rt_message::STATE_ERROR_CONTROLLER) {
        RCLCPP_ERROR(logger, "ROBOT ERROR. CODE: %d", rtMsgRecv_.header.msg_code);
        // std::unique_lock<std::mutex> lock(start_mutex);
        // start_signal.store(StartSignalIndicator::FAILURE);
        // start_condition.notify_one();
    }
    return CallbackReturn::FAILURE;
    

    // Wait for robot to become ready
    // start_signal.store(StartSignalIndicator::NONE);
    // std::unique_lock<std::mutex> lock(start_mutex);
    // start_condition.wait(lock, 
        // [&sig=start_signal]{return sig.load() != StartSignalIndicator::NONE;}
    // );

    // // For testing activation delays
    // uint16_t hw_start_sec_ = 5;
    // for (auto i = 0; i < hw_start_sec_; i++)
    // {

    //     RCLCPP_INFO(rclcpp::get_logger("MotomanHardware"), "%d seconds left...", hw_start_sec_ - i);
    //     rclcpp::sleep_for(std::chrono::seconds(1));
    // }


    // switch (start_signal.load()) {
    //     case StartSignalIndicator::SUCCESS: 
    //     {
    //         RCLCPP_INFO(logger, "Successfully activated!");
    //         return CallbackReturn::SUCCESS;
    //     }
    //     default:
    //     {
    //         RCLCPP_ERROR(logger, "Activation failure!");
    //         return CallbackReturn::ERROR;
    //     }
    // }

}


hardware_interface::CallbackReturn MotomanHardware::on_shutdown(const rclcpp_lifecycle::State & /* previous_state */)
{
    if(!_is_deactivated) {MotomanHardware::on_deactivate(this->get_state());}

    RCLCPP_WARN(logger, "Shutdown Hardware Interface ...");
    return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::return_type MotomanHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    // This startup condition is related to https://github.com/ros-controls/ros2_control/pull/1451 not beeing merged.
    // The flag 'robot_controller_initialized' can be removed as soon as the first read/write is done AFTER on_activation is finished!
    // This error will only show up if 'robot_description' is loaded through a topic.
    // RCLCPP_WARN_ONCE(logger, "first read");
    if (!robot_controller_initialized) {return hardware_interface::return_type::ERROR;}


    // fd_set server_rfds;
    // FD_ZERO(&server_rfds);
    // FD_SET(udp_socket_fd, &server_rfds);
    
    // block while waiting for response or timeout
    //retval = pselect(udp_socket_fd+1, &server_rfds, NULL, NULL, &udp_timeout, NULL);
    // #define UDP_TIMEOUT_MS 4
    // retval = poll(&pfds, (unsigned long)1, UDP_TIMEOUT_MS);
    // if(retval == -1) {
    //     RCLCPP_ERROR(logger, "[UDP] Recv failed. ERRNO: %d", errno);
    //     return hardware_interface::return_type::ERROR;
    // }
    // else if(retval == 0) {
    //     // Timeout
    //     RCLCPP_WARN(logger, "[UDP] Recv timeout %d. NW_S_CNTR: %ld", UDP_TIMEOUT_MS, _nw_success_counter);
    //     _nw_miss_counter++;
    // }
    // else {
    //     _nw_success_counter++;
    // }

    // if(pfds.revents != POLL_IN) {
    //     RCLCPP_WARN(logger, "[UDP] got event: %d", pfds.revents);
    // }
    
    // if(period.nanoseconds() > 4500000) {
    //     RCLCPP_INFO(logger, "[HW] read period: %ld ns", period.nanoseconds());
    // }


    // clear message
    memset(&rtMsgRecv_, 0, RT_MSG_STATE_SIZE);

    // receive UDP message
    bytesRecv = recv(udp_socket_fd, &rtMsgRecv_, RT_MSG_STATE_SIZE, 0);
    if(bytesRecv < 0)
    {
        RCLCPP_ERROR(logger, "UDP receive error: %d", errno);
        return hardware_interface::return_type::ERROR;
    }

    // Read current state
    if(rtMsgState_ != rtMsgRecv_.header.msg_state)
    {
        rtMsgState_ = rtMsgRecv_.header.msg_state;
        RCLCPP_WARN(logger, "Robot STATE: %d", rtMsgState_);
    }

    // Read current code
    if(rtMsgCode_ != rtMsgRecv_.header.msg_code)
    {
        rtMsgCode_ = rtMsgRecv_.header.msg_code;
        RCLCPP_WARN(logger, "Robot CODE: %d", rtMsgCode_);
    }

    // Set position and velocity to ROS2 control pipeline
    for(uint8_t i = 0; i < joints_size; i++)
    {
        hw_pos_cmd[i] = rtMsgRecv_.body.state[0].pos_cmd[i];
        hw_pos_set[i] = rtMsgRecv_.body.state[0].pos_set[i];
        hw_pos_fb[i] = rtMsgRecv_.body.state[0].pos_fb[i];
        hw_vel_cmd[i] = rtMsgRecv_.body.state[0].vel_cmd[i];
        hw_vel_set[i] = rtMsgRecv_.body.state[0].vel_set[i];
        hw_vel_fb[i] = rtMsgRecv_.body.state[0].vel_fb[i];
        hw_acc_cmd[i] = rtMsgRecv_.body.state[0].acc_cmd[i];
        hw_acc_set[i] = rtMsgRecv_.body.state[0].acc_set[i];
    }

    // Add data tamer here #TODO

    
    return hardware_interface::return_type::OK;
}


hardware_interface::return_type MotomanHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    // Crucial first check. Read and write are called before on_activate is finished!
    if (!robot_controller_initialized) {return hardware_interface::return_type::ERROR;}

    // Set header
    rtMsgSend_.header.msg_state = rtMsgRecv_.header.msg_state;
    rtMsgSend_.header.msg_code = udp_rt_message::CODE_UNDEFINED; // #TODO
    rtMsgSend_.header.msg_sequence = rtMsgRecv_.header.msg_sequence;

    // Set speed
    rtMsgSend_.body.command[0].INC_FACTOR = ROBOT_INC_MAX_FACTOR;
    rtMsgSend_.body.command[0].ACC_FACTOR = ROBOT_INC_ACC_FACTOR;

    for(uint8_t i = 0; i < joints_size; i++)
    {
        // #TODO handle multiple control groups
        
        // Handle NaN values
        if (std::isnan(hw_commands[i])) {
            hw_commands[i] = hw_cmd_prev[i];
            RCLCPP_WARN_ONCE(logger, "axis %d: got NaN value", i); 
        }

        rtMsgSend_.body.command[0].pos[i] = hw_commands[i];
        hw_pos_snd[i] = hw_commands[i];     // #TEST publish current command in state interface
        hw_cmd_prev[i] = hw_commands[i];

        // #TODO remove for new controller software
        // if(!initial_controller_commands && rtMsgSend_.body.command[0].pos[i] != hw_cmd_initial[i])
        // {
        //     initial_controller_commands = true;
        //     rtMsgSend_.header.msg_state = udp_rt_message::STATE_RUN_IN;
        // }

    }

    if (send(udp_socket_fd, &rtMsgSend_, RT_MSG_CMD_SIZE, MSG_CONFIRM) <0) {
        RCLCPP_ERROR(logger, "[UDP] send error: %d", errno);
        return hardware_interface::return_type::ERROR;
    }
    
    return hardware_interface::return_type::OK;
}


std::vector<hardware_interface::StateInterface> MotomanHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    // Main state interfaces
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_pos_set[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_vel_set[i]));

        // Additional position state interfaces
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, "pos_snd", &hw_pos_snd[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, "pos_cmd", &hw_pos_cmd[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, "pos_fdb", &hw_pos_fb[i]));

        // Additional velocity state interfaces
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, "vel_cmd", &hw_vel_cmd[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, "vel_fdb", &hw_vel_fb[i]));

        // Additional acceleration state interfaces
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, "acc_cmd", &hw_acc_cmd[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, "acc_set", &hw_acc_set[i]));
    }
    
    // state_interfaces.emplace_back(hardware_interface::StateInterface(
    //     info_.joints[i].name, "state_msg", &hw_state_msg[i]));
    

    return state_interfaces;
}


std::vector<hardware_interface::CommandInterface> MotomanHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (auto i = 0u; i < info_.joints.size(); i++)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands[i]));
    }

    return command_interfaces;
}


REQUEST_RETURN_TYPE MotomanHardware::send_tcp_motion_request(simple_message::SmCommandType command)
{
    // ------------------ SEND REQUEST ------------------
    // prepare tcp motion request
    simple_message::SimpleMsg motionReq, motionResp;
    motionReq.prefix.length = sizeof(simple_message::SmHeader) + sizeof(simple_message::SmBodyMotoMotionCtrl);
    motionReq.header.msgType = simple_message::SmMsgType::ROS_MSG_MOTO_MOTION_CTRL;
    motionReq.header.commType = simple_message::SmCommType::ROS_COMM_TOPIC;
    motionReq.header.replyType = simple_message::SmReplyType::ROS_REPLY_INVALID;
    motionReq.body.motionCtrl.command = command;
    
    size_t msg_len = motionReq.prefix.length + sizeof(simple_message::SmPrefix);
    //RCLCPP_INFO(logger, "TCP: sending %ld bytes", msg_len);

    // send tcp motion request
    bytesSend = send(tcp_socket_fd, &motionReq, msg_len, 0);
    if (bytesSend <= 0) {
        RCLCPP_ERROR(logger, "[TCP] Send failed. ERRNO: %d", errno);
        return REQUEST_RETURN_TYPE::ERROR;
    }

    // ------------------ RECEIVE RESPONSE ------------------
    // initialize file descriptor. rfds = read file descriptor(s)
    fd_set server_rfds;
    FD_ZERO(&server_rfds);
    FD_SET(tcp_socket_fd, &server_rfds);

    retval = pselect(tcp_socket_fd+1, &server_rfds, NULL, NULL, &tcp_timeout, NULL);
    if (retval == -1) {
        RCLCPP_ERROR(logger, "[TCP] Recv failed. ERRNO: %d", errno);
        return REQUEST_RETURN_TYPE::ERROR;
    }
    else if(retval == 0)
    {
        // Timeout
        RCLCPP_WARN(logger, "[TCP] Recv timeout %ld s", tcp_timeout.tv_sec);
        return REQUEST_RETURN_TYPE::ERROR;
    }

    // Data available to read
    bytesRecv = recv(tcp_socket_fd, &motionResp, sizeof(simple_message::SmHeader) + sizeof(simple_message::SmBodyMotoMotionReply), MSG_WAITALL);
    if(bytesRecv < 0)
    {
        RCLCPP_ERROR(logger, "[TCP] Recv failed. ERRNO: %d", errno);
        return REQUEST_RETURN_TYPE::ERROR;  
    }
    //RCLCPP_INFO(logger, "[TCP] Recv got result: %d\n", motionResp.body.motionReply.result);
    
    switch (motionResp.body.motionReply.result)
    {
    case simple_message::SmResultType::ROS_RESULT_SUCCESS:
        return REQUEST_RETURN_TYPE::SUCCESS;
        break;
    case simple_message::SmResultType::ROS_RESULT_BUSY:
        return REQUEST_RETURN_TYPE::ERROR;
        break;
    case simple_message::SmResultType::ROS_RESULT_FAILURE:
        return REQUEST_RETURN_TYPE::ERROR;
        break;
    default:
        break;
    }
    
    RCLCPP_WARN(logger, "[TCP] Recv unknown result: %d", motionResp.body.motionReply.result);
    return REQUEST_RETURN_TYPE::ERROR;
}


void MotomanHardware::set_robot_status_thread(THREAD_STATE state)
{
	switch (state) {
	case THREAD_STATE::START:
    {
        ros_status_executor_ptr = std::make_shared<rclcpp::executors::StaticSingleThreadedExecutor>();
        std::weak_ptr<rclcpp::executors::StaticSingleThreadedExecutor> weak_obj(ros_status_executor_ptr);
        run_robot_status_node.store(true);

		if (!robot_status_thread_ptr) {
            RCLCPP_INFO(logger, "starting robot_status_thread ...");

			robot_status_thread_ptr = std::unique_ptr<std::thread>(
				new std::thread(
					[
                        &ip_address = ip_address,
                        wk_ptr = weak_obj,
                        &run_thread = run_robot_status_node
                    ]()
					{
						auto node = std::make_shared<StateServer>(ip_address, &run_thread);

                        auto executor = wk_ptr.lock();
                        executor->add_node(node);
                        executor->spin();
                        executor->remove_node(node);
					}
				)
			);
		}
		break;
    }
	case THREAD_STATE::STOP:
    {
		if (robot_status_thread_ptr) {
            if (ros_status_executor_ptr) {
                run_robot_status_node.store(false);
                ros_status_executor_ptr->cancel();
			    robot_status_thread_ptr->join();
                // RCLCPP_INFO(logger, "robot_status_thread ended");
            }
            else RCLCPP_WARN(logger, "STATUS SERVER NODE ALREADY DEAD");
		}
		break;
    }
	default:
		break;
	}
}


// void MotomanHardware::run_robot_status_polling() {

// }

// hardware_interface::return_type MotomanHardware::prepare_command_mode_switch(
//     const std::vector<std::string> & /*start_interfaces*/, const std::vector<std::string> & /*stop_interfaces*/)
// {
//     RCLCPP_ERROR(logger, "prepare_command_mode_switch");
//     return hardware_interface::return_type::OK;
// }

// hardware_interface::return_type MotomanHardware::perform_command_mode_switch(
//     const std::vector<std::string> & /*start_interfaces*/, const std::vector<std::string> & /*stop_interfaces*/)
// {
//     RCLCPP_ERROR(logger, "perform_command_mode_switch");
//     return hardware_interface::return_type::OK;
// }


} // namespace motoman_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(motoman_hardware::MotomanHardware, hardware_interface::SystemInterface)