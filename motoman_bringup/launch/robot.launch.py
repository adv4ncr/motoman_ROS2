# source: https://github.com/frankaemika/franka_ros2/blob/develop/franka_bringup/launch/franka.launch.py 
# others: https://github.com/ICube-Robotics/iiwa_ros2/blob/main/iiwa_bringup/launch/iiwa.launch.py 
# others: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/main/ur_bringup/launch/ur10.launch.py


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    robot_ip_parameter_name = "robot_ip"
    mock_hardware_parameter_name = "use_mock_hardware"
    use_rviz_parameter_name = "use_rviz"
    namespace_parameter_name = "namespace"
    axis_increment_factor_parameter_name = "axis_increment_factor"
    axis_acceleration_factor_parameter_name = "axis_acceleration_factor"

    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    mock_hardware = LaunchConfiguration(mock_hardware_parameter_name)
    use_rviz = LaunchConfiguration(use_rviz_parameter_name)
    namespace = LaunchConfiguration(namespace_parameter_name)
    axis_increment_factor = LaunchConfiguration(axis_increment_factor_parameter_name)
    axis_acceleration_factor = LaunchConfiguration(axis_acceleration_factor_parameter_name)

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            robot_ip_parameter_name,
            description='Hostname or IP address of the robot.')
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            use_rviz_parameter_name,
            default_value="false",
            description='Visualize the robot in Rviz')
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'namespace',
            default_value='/',
            description='Namespace of launched nodes, useful for multi-robot setup. \
                         If changed than also the namespace in the controllers \
                         configuration needs to be updated. Expected format "<ns>/".',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            mock_hardware_parameter_name,
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            axis_increment_factor_parameter_name,
            default_value="0.5",
            description="Set increment factor for all robot axis.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            axis_acceleration_factor_parameter_name,
            default_value="0.05",
            description="Set acceleration factor for all robot axis.",
        )
    )

    robot_xacro_file = os.path.join(get_package_share_directory('motoman_description'), 'robots', 'hc10dt_b10.xacro')

    robot_description = ParameterValue( 
        Command(
            [FindExecutable(name='xacro'), ' ', robot_xacro_file,
            ' robot_ip:=', robot_ip,
            ' use_mock_hardware:=', mock_hardware,
            ' axis_increment_factor:=', axis_increment_factor,
            ' axis_acceleration_factor:=', axis_acceleration_factor,
            ]), 
        value_type=str)


    rviz_file = os.path.join(get_package_share_directory('motoman_description'), 'rviz',
                             'visualize_motoman.rviz')

    motoman_controllers = PathJoinSubstitution(
        [
            FindPackageShare('motoman_bringup'),
            'config',
            'controllers.yaml',
        ]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{"robot_description": robot_description}],
    )

    controller_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        # parameters=[motoman_controllers], #TODO remove 'robot_description' as parameter
        parameters=[{"robot_description": robot_description}, motoman_controllers],
        remappings=[
            ('/controller_manager/robot_description', 'robot_description'),
        ],
        output="screen",
        namespace=namespace,
        on_exit=Shutdown(),
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager',
                   [namespace, 'controller_manager']],
        output='screen',
        namespace=namespace,
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['joint_trajectory_controller', '--inactive', '--controller-manager', [namespace, 'controller_manager']],
        namespace=namespace,
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['--display-config', rviz_file],
        condition=IfCondition(use_rviz),
        namespace=namespace,
    )


    cartesian_motion_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["cartesian_motion_controller", "--inactive", "-c", "/controller_manager"],
    )


    motion_control_handle_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["motion_control_handle", "--inactive", "-c", "/controller_manager"],
    )
    
    robot_capture_handle_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robot_capture_handle", "-c", "/controller_manager"],
    )

    static_test_handle_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["StaticTestController", "--inactive", "-c", "/controller_manager"],
    )


    # ------------------ NODES ------------------
    nodes = [
        controller_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        static_test_handle_spawner,
        robot_capture_handle_spawner,
        # delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
