#ifndef STATIC_TEST_HPP
#define STATIC_TEST_HPP

#include <math.h>

#include <rclcpp/rclcpp.hpp>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "motoman_controllers/visibility_control.h"
#include "controller_interface/helpers.hpp"

#include "controller_interface/controller_interface.hpp"

#include "motoman_controllers/motion_generators.hpp"
#include "motoman_controllers_parameters.hpp"

namespace motoman_controllers
{

class StaticTest : public controller_interface::ControllerInterface
{
public:

    MOTOMAN_CONTROLLERS_PUBLIC
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    MOTOMAN_CONTROLLERS_PUBLIC
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    MOTOMAN_CONTROLLERS_PUBLIC
    controller_interface::CallbackReturn on_init() override;

    MOTOMAN_CONTROLLERS_PUBLIC
    controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

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

    std::shared_ptr<ParamListener> param_listener_;
    Params params_;

    std::vector<std::string> state_interface_types;
    //std::vector<std::string> command_interface_types;
    std::vector<std::string> joint_names;

    #define AXES 6
    #define AX_TEST 0 // [0..5]
    uint8_t _axs;
    u_int32_t _counter_val = 0, _counter_val_back = 0, _counter_axs = 0;

    #define time_span 2    // [s]

    std::array<double, 250*time_span> t_values;
    std::array<double, 250*time_span> ax_pos;
    std::array<double, 250*time_span> ax_vel;
    std::array<double, 250*time_span> ax_acc;

    std::vector<double> back_to_init_pos;

    // enum DriveMode {NONE, SINGLE, ALL, SEQUENCE};
    // DriveMode drive_mode = DriveMode::NONE;
    // bool state_entry = true;

};
} // namespace motoman_controllers
#endif // STATIC_TEST_HPP
