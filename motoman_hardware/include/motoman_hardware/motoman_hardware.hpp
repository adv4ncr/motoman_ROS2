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
#include "motoman_hardware/visibility_control.h"



namespace motoman_hardware
{
class MotomanHardware : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(MotomanHardware)
    
    MOTOMAN_HARDWARE_PUBLIC
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo & info) override;

    MOTOMAN_HARDWARE_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    MOTOMAN_HARDWARE_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    MOTOMAN_HARDWARE_PUBLIC
    hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

    MOTOMAN_HARDWARE_PUBLIC
    hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

    MOTOMAN_HARDWARE_PUBLIC
    hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

    MOTOMAN_HARDWARE_PUBLIC
    hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:

    //void send_tcp_motion_request();

    std::vector<double> hw_commands_;
    std::vector<double> hw_positions_;
    std::vector<double> hw_velocities_;
    size_t joints_size;

    // simple messages
    simple_message::SimpleMessage state_msg_;
    simple_message::SimpleMessage command_msg_;

    struct sockaddr_in udp_servaddr, tcp_servaddr;
    std::string ip_address;
    int udp_socket_fd, tcp_socket_fd;

    unsigned int sizeof_udp_servaddr;
    ssize_t bytesSend, bytesRecv;

};




} // namespace motoman_hardware

#endif //MOTOMAN_HARDWARE_HPP
