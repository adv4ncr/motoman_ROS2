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

    std::vector<double> hw_commands_;
    std::vector<double> hw_positions_;
    std::vector<double> hw_velocities_;

    // simple messages
    simple_message::SimpleMessage state_msg_;
    simple_message::SimpleMessage command_msg_;

    struct sockaddr_in udp_servaddr;
    std::string udp_ip_address;
    uint16_t udp_port;
    int udp_socket_fd;

    ssize_t bytesReceived;
    unsigned int sizeof_udp_servaddr;
 

};




} // namespace motoman_hardware

#endif //MOTOMAN_HARDWARE_HPP
