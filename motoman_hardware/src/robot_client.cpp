#include "motoman_hardware/robot_client.hpp"
#include "motoman_hardware/simple_message.hpp"
#include <cstdint>
#include <rclcpp/qos.hpp>
#include <sys/socket.h>

namespace motoman_hardware
{

RobotClient::RobotClient (std::string & ip_address, std::atomic_bool * flag_ptr)
  : Node("motoman_robot_client"),
  run_thread(flag_ptr)
{
  // Save ip address
  robot_ip_address = ip_address;

  // Connect server
  for (uint8_t i = 0; i < ROBOT_SOCKET_NUM; i++) {
    if (!connect_client(tcp_socket_fd[i], TCP_PORT_LIST[i])) {return;}
    pfds[i].fd = tcp_socket_fd[i];
    pfds[i].events = POLL_IN;
  }

  // Create publisher
  rclcpp::QoS _qos(1);
  _qos.reliable();
  _qos.transient_local();
  state_publisher = this->create_publisher<motoman_msgs::msg::RobotState>("state", _qos);

  // Create service


  // Running the seperate thread
  run_client();
}

bool RobotClient::connect_client(int& socket_fd, int port) {

  // creating tcp socket fd
  socket_fd = socket(AF_INET, SOCK_STREAM, 0);
  if (socket_fd < 0) {
    RCLCPP_ERROR(this->get_logger(), "tcp socket creation failed");
    //thread_state = THREAD_STATE::ERROR;
    return false;
  }

  // set tcp client struct
  bzero(&tcp_servaddr, sizeof(tcp_servaddr));
  tcp_servaddr.sin_family = AF_INET;
  inet_pton(AF_INET, robot_ip_address.c_str(), &(tcp_servaddr.sin_addr.s_addr));
  tcp_servaddr.sin_port = htons(port);

  // connect the client socket to server socket
  if (connect(socket_fd, (struct sockaddr*)&tcp_servaddr, sizeof(tcp_servaddr)) != 0) {
    RCLCPP_ERROR(this->get_logger(), "tcp failed to connect to port %d. ERRNO: %d", TCP_PORT_MOTION_COMMAND, errno);
    //thread_state = THREAD_STATE::ERROR;
    return false;
  }

  return true;
}

void RobotClient::run_client () 
{
  RCLCPP_INFO(this->get_logger(), "listening on state messages TCP port %d", TCP_PORT_STATE);
  RCLCPP_INFO(this->get_logger(), "listening on io messages TCP port %d", TCP_PORT_IO);


  while (*run_thread) {

    // Timeout [ms], -1 = wait forever
    retval = poll(pfds, (unsigned long)ROBOT_SOCKET_NUM, 1000);

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
    
    for (uint8_t i=0; i<ROBOT_SOCKET_NUM; i++) {

      // Data available to read
      if (pfds[i].revents & POLL_IN) {

        bytesRecv = recv(tcp_socket_fd[i], msgs.begin(), sizeof(simple_message::SimpleMsg), MSG_WAITALL);

        // Parse MSG
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
          RCLCPP_WARN(this->get_logger(), "UNKNOWN MSG_TYPE: %d", msgs.begin()->header.msgType);
        }

      } else {
        RCLCPP_WARN(this->get_logger(), "[TCP] got event: %d", pfds[i].revents);
      }

    }


  }

  RCLCPP_INFO(this->get_logger(), "exit");

  // Disconnect
  for (uint8_t i = 0; i < ROBOT_SOCKET_NUM; i++) close(tcp_socket_fd[i]);
}

} // namespace motoman_hardware