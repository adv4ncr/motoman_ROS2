# motoman_ROS2
Experimental [ros2_control](https://github.com/ros-controls/ros2_control) integration for Yaskawa Controller focused on low latency direct input. Also works with [moveit2](https://github.com/moveit/moveit2). Tested on YRC1000 and YRC1000u.

# Controller Driver
Based on https://github.com/tingelst/motoman .  
Use `MotoPlus for Visual Studio` or `MotoPlusIDE` to compile the controller code.

# Available State Interfaces
## Nomenclature: 
- `Robot controller` - the hardware robot controller (e.g. YRC1000)
## State Interfaces
- `<state_interface name="position">` - joint position sent to the robot controller. Since the robot is a commanded system (as opposed to a controlled system), the last command sent is used as the position state. This works well in practice with the disadvantage that the inherent dead time of >= 40 ms is not reflected in the state. Use the `fdb` interface for this.
- `<state_interface name="pos_set">` - joint position set on the robot controller
- `<state_interface name="pos_cmd">` - joint position commanded to the robot controller (same as the "position" interface, just reflected from the robot controller) 
- `<state_interface name="pos_fdb">` - real joint position measured by the robot controller
- `<state_interface name="vel_cmd">` - joint velocity commanded to the robot controller
- `<state_interface name="velocity">` - joint velocity set on the robot controller
- `<state_interface name="vel_fdb">` - real joint velocity measured by the robot controller
- `<state_interface name="acc_cmd">` - joint acceleration commanded to the robot controller
- `<state_interface name="acc_set">` - joint acceleration set on the robot controller

# Working principle
The individual joint positions are sent (`snd`) to the robot controller.  
A dedicated algorithm running on the robot controller checks the commanded (`cmd`) position against the configured maximum values (see robot URDF).  
The algorithm sets (`set`) the resulting increments to the robot hardware.  
The robot controller measures the actual position and velocity using its feedback API (`fdb`).