#ifndef STATIC_TEST_HPP
#define STATIC_TEST_HPP

#include <math.h>

#include <rclcpp/rclcpp.hpp>

#include "rclcpp/logging.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"


#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "motoman_controllers/visibility_control.h"
#include "controller_interface/helpers.hpp"

#include "forward_command_controller/forward_command_controller.hpp"

namespace motoman_controllers
{
using CmdType = std_msgs::msg::Float64MultiArray;

class StaticTest : public forward_command_controller::ForwardCommandController
{
public:
    MOTOMAN_CONTROLLERS_PUBLIC
    StaticTest();
    
    MOTOMAN_CONTROLLERS_PUBLIC
    ~StaticTest() = default;

    // MOTOMAN_CONTROLLERS_PUBLIC
    // controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    MOTOMAN_CONTROLLERS_PUBLIC
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    MOTOMAN_CONTROLLERS_PUBLIC
    controller_interface::CallbackReturn on_init() override;

    // MOTOMAN_CONTROLLERS_PUBLIC
    // controller_interface::CallbackReturn on_configure(
    // const rclcpp_lifecycle::State & previous_state) override;

    MOTOMAN_CONTROLLERS_PUBLIC
    controller_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;

    // MOTOMAN_CONTROLLERS_PUBLIC
    // controller_interface::CallbackReturn on_deactivate(
    //     const rclcpp_lifecycle::State & previous_state) override;

    MOTOMAN_CONTROLLERS_PUBLIC
    controller_interface::return_type update(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:


    // const std::vector<std::string> allowed_interface_types_ = {
    //     hardware_interface::HW_IF_POSITION,
    //     // hardware_interface::HW_IF_VELOCITY,
    //     // hardware_interface::HW_IF_ACCELERATION,
    //     // hardware_interface::HW_IF_EFFORT,
    // };

private:

    const float pulseToRad[8] =
    {
        153054.453125,
        153054.453125,
        134496.984375,
        92770.187500,
        92770.187500,
        92770.187500,
        0.000000,
        0.000000
    };

    const uint32_t maxIncrement[8] = 
    {
        1389,
        1389,
        1690,
        1165,
        1619,
        1619,
        0,
        0
    };

    #define period_len 500
    #define values_size period_len*3+1
    #define axes 6
    #define scale_factor 0.75

    double factors[axes];
    double values[values_size][axes];

	const double step = M_PI*2/period_len;
	double increment;
    uint8_t _axs;

    double start_values[axes];
    u_int32_t counter = 0;
    u_int32_t _sets = 0;

};
} // namespace motoman_controllers
#endif // STATIC_TEST_HPP#include 