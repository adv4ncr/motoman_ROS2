cmake_minimum_required(VERSION 3.8)
project(motoman_hardware)

set(CMAKE_EXPORT_COMPILE_COMMANDS on)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
# find_package(fastrtps REQUIRED)
# find_package(fastcdr REQUIRED)
find_package(motoman_description REQUIRED)
find_package(motoman_msgs REQUIRED)

## Add RobotState message lib
# message(STATUS "Configuring RobotState...")
# add_library(RobotState_lib src/RobotState.cxx)
# target_link_libraries(RobotState_lib fastcdr fastrtps)
# target_include_directories(RobotState_lib PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)


## Add main lib
add_library(
  ${PROJECT_NAME}
  SHARED
  src/motoman_hardware.cpp
  src/status_server.cpp
  # src/RobotStatePubSubTypes.cxx
  # src/RobotState.cxx
)

# target_link_libraries(
#   ${PROJECT_NAME}
#   fastrtps
#   fastcdr
# )

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)

ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  motoman_description
  motoman_msgs
)

#get_target_property(OUT ${PROJECT_NAME} LINK_LIBRARIES)
#message(WARN ${OUT})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "MOTOMAN_HARDWARE_BUILDING_DDL")

pluginlib_export_plugin_description_file(hardware_interface motoman_hardware.xml)

# INSTALL
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
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

## EXPORTS
ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)


ament_package()