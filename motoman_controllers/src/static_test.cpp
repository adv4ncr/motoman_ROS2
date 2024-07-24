#include <cstdio>
#include <string>

#include "motoman_controllers/static_test.hpp"

namespace motoman_controllers
{


controller_interface::CallbackReturn StaticTest::on_init()
{

	try {
		param_listener_ = std::make_shared<ParamListener>(get_node());
		params_ = param_listener_->get_params();
	}
	catch (const std::exception & e) {
		RCLCPP_ERROR(
			get_node()->get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
		return CallbackReturn::ERROR;
	}

	return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn StaticTest::on_configure(const rclcpp_lifecycle::State & /*prev_state*/)
{
	if (!param_listener_) {
		RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
		return controller_interface::CallbackReturn::ERROR;
	}

	// Update the dynamic map parameters
	param_listener_->refresh_dynamic_parameters();
	// Check provided parameters
	params_ = param_listener_->get_params();


	// Get names of actuated joints
	

	joint_names = get_node()->get_parameter("joints").as_string_array();
	if (joint_names.empty()) {
		RCLCPP_ERROR(get_node()->get_logger(), "joints array is empty");
		return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
	}

	// Get the state_interfaces parameter
	state_interface_types = get_node()->get_parameter("state_interfaces").as_string_array();
	if (state_interface_types.empty()) {
		RCLCPP_ERROR(get_node()->get_logger(), "No state_interfaces specified");
		return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
	}
	
	// Get the command_interfaces parameter -> FIXED to: hardware_interface::HW_IF_POSITION
	// command_interface_types = get_node()->get_parameter("command_interfaces").as_string_array();
	// if (command_interface_types.empty()) {
	// 	RCLCPP_ERROR(get_node()->get_logger(), "No command_interfaces specified");
	// 	return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
	// }



	// ------------------ MOTION TEST ------------------
	
	// Set time values
	double _t_inc = 0;
	for(auto i = 0u; i<t_values.size(); i++) {
		t_values[i] = _t_inc;
		_t_inc += 1/250.;
	}

	// Set axis values

	constexpr double _p0 = 25 * M_PI/180.;
	constexpr double _v0 = 0 * M_PI/180.;
	constexpr double _a0 = 1250 * M_PI/180.;

	motion_generators::Step gen(_p0);
	// motion_generators::Ramp gen(_p0, _v0, _a0);
	//motion_generators::Sine gen(_p0, _v0, _a0);
	//motion_generators::RealData gen("my_data.csv");

	
	for(auto i = 0u; i<ax_pos.size(); i++) {
		gen.get_values(t_values[i], ax_pos[i], ax_vel[i], ax_acc[i]);
	}

	// Drive back to initial position 0
	double _p0_back = ax_pos.back();
	double _v_back = _p0_back > 0 ? -25 * M_PI/180. : 25 * M_PI/180.;
	//std::cout << "p0: " << _p0_back << " v: " << _v_back << std::endl;
	motion_generators::Ramp gen_back(_p0_back, _v_back, 0);
	double t_back = 0, ax_pos_back, ax_vel_back, ax_acc_back;
	for(;;) {
		if(_p0_back == 0.) break;
		else if(back_to_init_pos.size() > 5000) break;
		gen_back.get_values(t_back, ax_pos_back, ax_vel_back, ax_acc_back);
		if((_p0_back>0 && ax_pos_back<0) || (_p0_back<0 && ax_pos_back>0) ) break;
		back_to_init_pos.push_back(ax_pos_back);
		t_back+=1/250.;
	}
	back_to_init_pos.push_back(0.);

	RCLCPP_WARN(get_node()->get_logger(), "[DRIVE_BACK] trajectory size: %ld", back_to_init_pos.size());



	return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration StaticTest::state_interface_configuration() const
{
	controller_interface::InterfaceConfiguration conf;
	conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;

	conf.names.reserve(joint_names.size() * state_interface_types.size());

	for (const auto & interface_type : state_interface_types) {
		for (const auto & joint_name : joint_names) {
			conf.names.push_back(joint_name + "/" + interface_type);
		}
	}

	return conf;
}

controller_interface::InterfaceConfiguration StaticTest::command_interface_configuration() const
{
	controller_interface::InterfaceConfiguration conf;
	conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;

	conf.names.reserve(joint_names.size());

	for (const auto & joint_name : joint_names) {
		conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
		
	}

	return conf;
}

controller_interface::CallbackReturn StaticTest::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
	std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> ordered_command_interfaces;

	if (!controller_interface::get_ordered_interfaces(
			command_interfaces_, joint_names, 
			hardware_interface::HW_IF_POSITION, ordered_command_interfaces) ||
		joint_names.size() != ordered_command_interfaces.size())
	{
		RCLCPP_ERROR(
			get_node()->get_logger(), "Expected %zu command interfaces, got %zu",
			joint_names.size(), ordered_command_interfaces.size());
		return controller_interface::CallbackReturn::ERROR;
	}

	// number of axis validation
	if(AXES != state_interfaces_.size())
	{
		RCLCPP_ERROR(
			get_node()->get_logger(), "Expected %d state interfaces, got %zu!",
			AXES, state_interfaces_.size());
		return controller_interface::CallbackReturn::ERROR;
	}


	// check current position
	// for (auto index = 0u; index < 6; ++index)
	// {
	// 	auto pos = state_interfaces_[index].get_value();
	// 	auto name = state_interfaces_[index].get_name();
	// 	RCLCPP_INFO(rclcpp::get_logger("StaticTest"), "[STATIC_TEST] state_interfaces_[%s]: %f", name.c_str(), pos);
	// 	start_values[index] = state_interfaces_[index].get_value();
	// }



	// // intit variables
	// increment = 0.0;

	// // set scaling factors
    // for(_axs = 0; _axs < axes; _axs++)
    // {
    //     factors[_axs] = maxIncrement[_axs] / (step*pulseToRad[_axs]) * scale_factor;
    //     //RCLCPP_INFO(rclcpp::get_logger("StaticTest"), "[STATIC_TEST] factor[%d]: %lf", _axs, factors[_axs]);
    // }

	// // calculate values
	// for(auto _i = 0u; _i < values_size; _i++)
	// {
    //     for(_axs = 0; _axs < axes; _axs++)
    //     {
	// 	    values[_i][_axs] = (-cos(increment)+1) * factors[_axs];
    //     }
    //     increment += step;
	// }



    // //get max values
    // for(_axs = 0; _axs < axes ; _axs++)
    // {
    //     long pos = lround(M_PI_2/step);
    //     double _slope = (values[pos+1][_axs] - values[pos][_axs])*pulseToRad[_axs];
    //     RCLCPP_INFO(rclcpp::get_logger("StaticTest"), "[STATIC_TEST] maxIncrement[%d]: %lf", _axs, _slope);
    // }



    RCLCPP_INFO(get_node()->get_logger(), "activate successful");

	return controller_interface::CallbackReturn::SUCCESS;
}



controller_interface::return_type StaticTest::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
	// // Drive sequentially
	// if(_counter_axs < 6)
	// {
	// 	// Set signal to hardware interface
	// 	if(_counter_val < t_values.size())
	// 	{
	// 		for (_axs = 0; _axs < AXES; _axs++)
	// 		{
	// 			if(_axs==_counter_axs) // set test axis
	// 			{
	// 				command_interfaces_[_axs].set_value(ax_pos[_counter_val]);
	// 			}
	// 		}
	// 		_counter_val++;
	// 	}
	// 	// Drive back
	// 	else if(_counter_val_back < back_to_init_pos.size())
	// 	{
	// 		for (_axs = 0; _axs < AXES; _axs++)
	// 		{
	// 			if(_axs==_counter_axs) // set test axis
	// 			{
	// 				command_interfaces_[_axs].set_value(back_to_init_pos[_counter_val_back]);
	// 			}
	// 		}
	// 		_counter_val_back++;	
	// 	}
	// 	else
	// 	{
	// 		_counter_val = 0;
	// 		_counter_val_back = 0;
	// 		_counter_axs++;
	// 	}
	// }
	// else

	// Drive all at once 
	
	// Set signal to hardware interface
	if(_counter_val < t_values.size())
	{
		for (_axs = 0; _axs < AXES; _axs++)
		{
			command_interfaces_[_axs].set_value(ax_pos[_counter_val]);
		}
		_counter_val++;
	}
	// Drive back
	else if(_counter_val_back < back_to_init_pos.size())
	{
		for (_axs = 0; _axs < AXES; _axs++)
		{
			command_interfaces_[_axs].set_value(back_to_init_pos[_counter_val_back]);
		}
		_counter_val_back++;	
	}
	else return controller_interface::return_type::ERROR;
	
	return controller_interface::return_type::OK;
}


} // motoman_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(motoman_controllers::StaticTest, controller_interface::ControllerInterface)



