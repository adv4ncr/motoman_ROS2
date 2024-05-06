#ifndef __STATUS_SERVER_HPP
#define __STATUS_SERVER_HPP

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

// enum class NODE_STATE {
//     INIT,
//     RUN,
//     STOP,
// };

class StateServer : public rclcpp::Node
{
public:
    StateServer (std::string & ip_address, std::atomic_bool * flag_ptr);
    //THREAD_STATE node_thread_state;

private:
    void run();

        // Robot status thread - read TCP messages from controller

    std::array<simple_message::SimpleMsg, 2> msgs;

    // tcp stack variables                        
    struct sockaddr_in tcp_servaddr;
    int retval, bytesRecv, tcp_socket_fd;
    pollfd pfds;

    // ROS2 publisher
    rclcpp::Publisher<motoman_msgs::msg::RobotState>::SharedPtr state_publisher;
    motoman_msgs::msg::RobotState state_msg;
    //std::atomic<NODE_STATE> * node_state;
    std::atomic_bool * run_thread;

};

} // namespace motoman_hardware

#endif // __STATUS_SERVER_HPP