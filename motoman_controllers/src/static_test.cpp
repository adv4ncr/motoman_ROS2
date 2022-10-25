#include <cstdio>
#include <string>

#include "motoman_controllers/static_test.hpp"

namespace motoman_controllers
{

StaticTest::StaticTest()
: forward_command_controller::ForwardCommandController()
{
	interface_name_ = hardware_interface::HW_IF_POSITION;
}

controller_interface::CallbackReturn StaticTest::on_init()
{

	auto ret = forward_command_controller::ForwardCommandController::on_init();
	if (ret != CallbackReturn::SUCCESS)
	{
		return ret;
	}

	try
	{
		// Explicitly set the interface parameter declared by the forward_command_controller
		// to match the value set in the StaticTest constructor.
		get_node()->set_parameter(
			rclcpp::Parameter("interface_name", hardware_interface::HW_IF_POSITION));
	}
	catch (const std::exception & e)
	{
		fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
		return CallbackReturn::ERROR;
	}


	return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration StaticTest::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names = command_interface_types_; // #TODO dirty test. Just works, because both interface_names are 'position'
  return conf;
}

controller_interface::CallbackReturn StaticTest::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
	std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> ordered_command_interfaces;
	if (!controller_interface::get_ordered_interfaces(
		command_interfaces_, command_interface_types_, std::string(""), ordered_command_interfaces) ||
		command_interface_types_.size() != ordered_command_interfaces.size())
	{
		RCLCPP_ERROR(
			get_node()->get_logger(), "Expected %zu command interfaces, got %zu",
			command_interface_types_.size(), ordered_command_interfaces.size());
		return controller_interface::CallbackReturn::ERROR;
	}

	std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> ordered_state_interfaces;
	if (!controller_interface::get_ordered_interfaces(
		state_interfaces_, command_interface_types_, std::string(""), ordered_state_interfaces) ||
		command_interface_types_.size() != ordered_state_interfaces.size())
	{
		RCLCPP_ERROR(
			get_node()->get_logger(), "Expected %zu state interfaces, got %zu",
			command_interface_types_.size(), ordered_state_interfaces.size());
		return controller_interface::CallbackReturn::ERROR;
	}

	// reset command buffer if a command came through callback when controller was inactive
	rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);

	// number of axis validation
	if(axes != state_interfaces_.size())
	{
		RCLCPP_ERROR(
			get_node()->get_logger(), "Expected %d state interfaces, got %zu",
			axes, state_interfaces_.size());
		return controller_interface::CallbackReturn::ERROR;
	}

	// check current position
	for (auto index = 0u; index < state_interfaces_.size(); ++index)
	{
		auto pos = state_interfaces_[index].get_value();
		auto name = state_interfaces_[index].get_name();
		RCLCPP_INFO(rclcpp::get_logger("StaticTest"), "[STATIC_TEST] state_interfaces_[%s]: %f", name.c_str(), pos);
		start_values[index] = state_interfaces_[index].get_value();
	}



	// ------------------ MOTION TEST ------------------
	// using cosinus wave
	
	// intit variables
	increment = 0.0;

	// set scaling factors
    for(_axs = 0; _axs < axes; _axs++)
    {
        factors[_axs] = maxIncrement[_axs] / (step*pulseToRad[_axs]) * scale_factor;
        //RCLCPP_INFO(rclcpp::get_logger("StaticTest"), "[STATIC_TEST] factor[%d]: %lf", _axs, factors[_axs]);
    }

	// calculate values
	for(auto _i = 0u; _i < values_size; _i++)
	{
        for(_axs = 0; _axs < axes; _axs++)
        {
		    values[_i][_axs] = (-cos(increment)+1) * factors[_axs];
        }
        increment += step;
	}



    //get max values
    for(_axs = 0; _axs < axes ; _axs++)
    {
        long pos = lround(M_PI_2/step);
        double _slope = (values[pos+1][_axs] - values[pos][_axs])*pulseToRad[_axs];
        RCLCPP_INFO(rclcpp::get_logger("StaticTest"), "[STATIC_TEST] maxIncrement[%d]: %lf", _axs, _slope);
    }



    RCLCPP_INFO(get_node()->get_logger(), "activate successful");

	return controller_interface::CallbackReturn::SUCCESS;
}



controller_interface::return_type StaticTest::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{


	// Set signal to hardware interface
	if(counter < values_size)
	{
		for (_axs = 0; _axs < axes; _axs++)
		{
			//if(_axs > 3) // set test axis
			{
				command_interfaces_[_axs].set_value(start_values[_axs] + values[counter][_axs]);
			}
		}
		counter++;
	}


	// if ((*joint_commands)->data.size() != command_interfaces_.size())
	// {
	//   RCLCPP_ERROR_THROTTLE(
	//     get_node()->get_logger(), *(get_node()->get_clock()), 1000,
	//     "command size (%zu) does not match number of interfaces (%zu)",
	//     (*joint_commands)->data.size(), command_interfaces_.size());
	//   return controller_interface::return_type::ERROR;
	// }


	// for (auto index = 0u; index < command_interfaces_.size(); ++index)
	// {
	//   command_interfaces_[index].set_value((*joint_commands)->data[index]);
	// }



	return controller_interface::return_type::OK;
}


} // motoman_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
	motoman_controllers::StaticTest, controller_interface::ControllerInterface)



