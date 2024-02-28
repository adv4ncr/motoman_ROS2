cmake_minimum_required(VERSION 3.8)
project(motoman_controllers)

set(CMAKE_EXPORT_COMPILE_COMMANDS on)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  # set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
    rclcpp
    rclcpp_lifecycle
    #forward_command_controller
    pluginlib
    controller_interface
    generate_parameter_library
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

#find_package(controller_interface REQUIRED)

add_library(motoman_controllers SHARED
  src/static_test.cpp
)

# Generate parameters from yml
generate_parameter_library(motoman_controllers_parameters
  src/motoman_controllers_parameters.yml
)

target_include_directories(motoman_controllers PRIVATE include)

ament_target_dependencies(motoman_controllers
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(motoman_controllers
  # Link parameters to controller library
  motoman_controllers_parameters
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(motoman_controllers PRIVATE "MOTOMAN_CONTROLLERS_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface motoman_controllers_plugins.xml)

# add_executable(static_test src/static_test.cpp)
# target_include_directories(static_test PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(static_test PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
# ament_target_dependencies(
#   static_test
#   "rclcpp"
#   "forward_command_controller"
# )

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
    motoman_controllers
    motoman_controllers_parameters
  EXPORT export_motoman_controllers
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# install(TARGETS static_test
#   DESTINATION lib/motoman_controllers)

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

# Exports
ament_export_targets(export_motoman_controllers HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
