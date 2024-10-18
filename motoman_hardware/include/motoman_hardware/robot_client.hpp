#ifndef __STATUS_SERVER_HPP
#define __STATUS_SERVER_HPP

#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include "motoman_hardware/simple_message.hpp"
#include "motoman_msgs/msg/robot_state.hpp"

#include <netinet/in.h>
#include <sys/socket.h>
#include <poll.h>
#include <arpa/inet.h>

namespace motoman_hardware
{

enum class THREAD_STATE {
  INIT,
  START,
  STOP,
  RUN,
  ERROR,
};

class RobotClient : public rclcpp::Node
{
public:
  RobotClient (std::string & ip_address, std::atomic_bool * flag_ptr);
  //THREAD_STATE node_thread_state;

private:
  bool connect_client(int& socket_fd, int port);
  void run_client();

  std::array<simple_message::SimpleMsg, 2> msgs;

  // tcp stack variables        
  std::string robot_ip_address;                
  struct sockaddr_in tcp_servaddr;
  int retval, bytesRecv;
  static constexpr uint8_t ROBOT_SOCKET_NUM = 2;
  int tcp_socket_fd[ROBOT_SOCKET_NUM];
  pollfd pfds[ROBOT_SOCKET_NUM];
  static constexpr int TCP_PORT_LIST[ROBOT_SOCKET_NUM] = {TCP_PORT_STATE, TCP_PORT_IO};


  // ROS2 publisher
  rclcpp::Publisher<motoman_msgs::msg::RobotState>::SharedPtr state_publisher;
  motoman_msgs::msg::RobotState state_msg;
  //std::atomic<NODE_STATE> * node_state;
  std::atomic_bool * run_thread;

};

} // namespace motoman_hardware

#endif // __STATUS_SERVER_HPP