cmake_minimum_required(VERSION 3.8)
project(motoman_hc10_moveit_servo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Component Nodes (Shared libraries) ############################
set(SERVO_CONTROLLER_INPUT servo_controller_input)

# set dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  sensor_msgs
  geometry_msgs
  control_msgs
  moveit_msgs
  std_msgs
  std_srvs
  rclcpp_components
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_components REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

#####################
## Component Nodes ##
#####################

# Add executable for using a controller
add_library(${SERVO_CONTROLLER_INPUT} SHARED src/joystick_servo_example.cpp)
ament_target_dependencies(${SERVO_CONTROLLER_INPUT} ${THIS_PACKAGE_INCLUDE_DEPENDS})
rclcpp_components_register_nodes(${SERVO_CONTROLLER_INPUT} "moveit_servo::JoyToServoPub")


# Install Libraries
install(
  TARGETS
    # ${SERVO_LIB_NAME}
    # ${SERVO_LIB_NAME}_parameters
    # ${POSE_TRACKING}
    # ${SERVO_COMPONENT_NODE}
    ${SERVO_CONTROLLER_INPUT}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)


install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
