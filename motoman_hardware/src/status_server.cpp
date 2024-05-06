#include "motoman_hardware/status_server.hpp"
#include <rclcpp/qos.hpp>

namespace motoman_hardware
{

StateServer::StateServer (std::string & ip_address, std::atomic_bool * flag_ptr)
    : Node("robot_state_server"),
    run_thread(flag_ptr)
{
    // RCLCPP_INFO_STREAM(this->get_logger(), "robot_state_thread_id: " << std::this_thread::get_id());


    // creating tcp socket fd
    tcp_socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (tcp_socket_fd < 0) {
        RCLCPP_ERROR(this->get_logger(), "tcp socket creation failed");
        //thread_state = THREAD_STATE::ERROR;
        return;
    }

    // set tcp client struct
    bzero(&tcp_servaddr, sizeof(tcp_servaddr));
    tcp_servaddr.sin_family = AF_INET;
    inet_pton(AF_INET, ip_address.c_str(), &(tcp_servaddr.sin_addr.s_addr));
    tcp_servaddr.sin_port = htons(TCP_PORT_STATE);

    // connect the client socket to server socket
    if (connect(tcp_socket_fd, (struct sockaddr*)&tcp_servaddr, sizeof(tcp_servaddr)) != 0) {
        RCLCPP_ERROR(this->get_logger(), "tcp failed to connect to port %d. ERRNO: %d", TCP_PORT_MOTION_COMMAND, errno);
        //thread_state = THREAD_STATE::ERROR;
        return;
    }

    pfds.fd = tcp_socket_fd;
    pfds.events = POLL_IN;

    // Create publisher
    rclcpp::QoS _qos(1);
    _qos.reliable();
    _qos.transient_local();
    state_publisher = this->create_publisher<motoman_msgs::msg::RobotState>("robot_state", _qos);

    // Running the seperate thread
    run();
}

void StateServer::run () 
{
    RCLCPP_INFO(this->get_logger(), "listening on robot_state messages TCP port %d", TCP_PORT_STATE);
    while (*run_thread) {

        // Timeout [ms], -1 = wait forever
        retval = poll(&pfds, (unsigned long)1, 1000);

        if (retval == -1) {
            RCLCPP_ERROR(this->get_logger(), "[TCP] Recv failed. ERRNO: %d", errno);
            return;
        }
        else if (retval == 0) {
            // Timeout
            continue;
        }
        else {
            // Success
        }
        
        if (pfds.revents != POLL_IN) {
            RCLCPP_WARN(this->get_logger(), "[TCP] got event: %d", pfds.revents);
        }

        // Data available to read
        bytesRecv = recv(tcp_socket_fd, msgs.begin(), 
            sizeof(simple_message::SmHeader) + sizeof(simple_message::SmPrefix) + sizeof(simple_message::SmBodyRobotStatus), 
            MSG_WAITALL);
        
        if(bytesRecv < 0) {
            RCLCPP_ERROR(this->get_logger(), "[TCP] Recv failed. ERRNO: %d", errno);
        }
        else if (msgs.begin()->header.msgType == simple_message::SmMsgType::ROS_MSG_ROBOT_STATUS) {
            
            // #TODO

            // RCLCPP_INFO(this->get_logger(), "-----------------");
            // RCLCPP_INFO(this->get_logger(), "direct_in: %d",
            //     msgs.begin()->body.robotStatus.input_direct_in);
            // RCLCPP_INFO(this->get_logger(), "drives_powered: %d",
            //     msgs.begin()->body.robotStatus.drives_powered);
            // RCLCPP_INFO(this->get_logger(), "e_stopped: %d",
            //     msgs.begin()->body.robotStatus.e_stopped);
            // RCLCPP_INFO(this->get_logger(), "in_error %d",
            //     msgs.begin()->body.robotStatus.in_error);
            // RCLCPP_INFO(this->get_logger(), "in_motion %d",
            //     msgs.begin()->body.robotStatus.in_motion);
            // RCLCPP_INFO(this->get_logger(), "mode %d",
            //     msgs.begin()->body.robotStatus.mode);
            // RCLCPP_INFO(this->get_logger(), "motion_possible %d",
            //     msgs.begin()->body.robotStatus.motion_possible);
            
            state_msg.input_direct_in = msgs.begin()->body.robotStatus.input_direct_in;
            state_publisher->publish(state_msg);
        }
        else {
            RCLCPP_WARN(this->get_logger(), "UNKNOWN STATE MSG_TYPE: %d", msgs.begin()->header.msgType);
        }
    }

    RCLCPP_WARN(this->get_logger(), "Exit State Server");

    // Disconnect
    close(tcp_socket_fd);
}

} // namespace motoman_hardware