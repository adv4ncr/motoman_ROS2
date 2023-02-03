#ifndef MOTOMAN_HARDWARE_HPP
#define MOTOMAN_HARDWARE_HPP



#include <rclcpp/rclcpp.hpp>
//#include <rclcpp/macros.hpp>
//#include <angles/angles.h>


#include <hardware_interface/system_interface.hpp>
#include "hardware_interface/types/hardware_interface_type_values.hpp"


#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include "motoman_hardware/simple_message.hpp"
#include "motoman_hardware/udp_rt_protocol.h"
#include "motoman_hardware/visibility_control.h"
// #include "motoman_description/msg/robot_state.hpp"


namespace motoman_hardware
{

#define TCP_TIMEOUT 1 // in s
#define UDP_TIMEOUT 500000 // in ns

class MotomanHardware : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(MotomanHardware)
    
    //UNCONFIGURED (on_init, on_cleanup):
    //Hardware is initialized but communication is not started and therefore no interface is available.

    MOTOMAN_HARDWARE_PUBLIC
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo & info) override;

    MOTOMAN_HARDWARE_PUBLIC
    hardware_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State & previous_state) override;

    // INACTIVE (on_configure, on_deactivate):
    //   Communication with the hardware is started and it is configured.
    //   States can be read and non-movement hardware interfaces commanded.
    //   Hardware interfaces for movement will NOT be available.
    //   Those interfaces are: HW_IF_POSITION, HW_IF_VELOCITY, HW_IF_ACCELERATION, and HW_IF_EFFORT.

    MOTOMAN_HARDWARE_PUBLIC
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

    MOTOMAN_HARDWARE_PUBLIC
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;
    
    // ACTIVE (on_activate):
    //   Power circuits of hardware are active and hardware can be moved, e.g., brakes are disabled.
    //   Command interfaces for movement are available and have to be accepted.
    //   Those interfaces are: HW_IF_POSITION, HW_IF_VELOCITY, HW_IF_ACCELERATION, and HW_IF_EFFORT.

    MOTOMAN_HARDWARE_PUBLIC
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;

    // FINALIZED (on_shutdown):
    //   Hardware interface is ready for unloading/destruction.
    //   Allocated memory is cleaned up.
    MOTOMAN_HARDWARE_PUBLIC
    hardware_interface::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State & previous_state) override;

    MOTOMAN_HARDWARE_PUBLIC

    MOTOMAN_HARDWARE_PUBLIC
    hardware_interface::return_type read(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

    MOTOMAN_HARDWARE_PUBLIC
    hardware_interface::return_type write(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

    MOTOMAN_HARDWARE_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    MOTOMAN_HARDWARE_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;


private:

    int send_tcp_request(simple_message::SmCommandType command);

    std::vector<double> hw_commands;
    std::vector<double> hw_pos_cmd;
    std::vector<double> hw_pos_set;
    std::vector<double> hw_pos_fb;
    std::vector<double> hw_vel_set;
    std::vector<double> hw_vel_fb;

    std::vector<double> hw_state_msg;

    bool init_hw_commands;
    size_t joints_size;

    // RT message variables
    udp_rt_message::RtMsg rtMsgRecv_, rtMsgSend_;
    udp_rt_message::RtMsgState rtMsgState_;
    udp_rt_message::RtMsgCode rtMsgCode_;

    // Network communication
    struct sockaddr_in udp_servaddr, tcp_servaddr;
    std::string ip_address;
    int udp_socket_fd, tcp_socket_fd;
    unsigned int sizeof_udp_servaddr;
    
    // Network file descriptor handling
    //fd_set server_rfds;
    struct timespec tcp_timeout, udp_timeout;
    int retval;

    // Network send /recv bytes
    ssize_t bytesSend, bytesRecv;


};




} // namespace motoman_hardware

#endif //MOTOMAN_HARDWARE_HPP
