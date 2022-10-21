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


def generate_launch_description():
    robot_ip_parameter_name = 'robot_ip'
    load_gripper_parameter_name = 'load_gripper'
    use_rviz_parameter_name = 'use_rviz'

    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    load_gripper = LaunchConfiguration(load_gripper_parameter_name)
    use_rviz = LaunchConfiguration(use_rviz_parameter_name)

    # HC10 hardcoded #TODO
    motoman_xacro_file = os.path.join(get_package_share_directory('motoman_description'), 'robots', 'hc10.xacro')

    robot_description = Command(
        [FindExecutable(name='xacro'), ' ', motoman_xacro_file, #' hand:=', load_gripper,
         ' robot_ip:=', robot_ip, #' use_fake_hardware:=', use_fake_hardware,
         ' fake_sensor_commands:=', #fake_sensor_commands
        ])

    rviz_file = os.path.join(get_package_share_directory('motoman_description'), 'rviz',
                             'visualize_motoman.rviz')

    motoman_controllers = PathJoinSubstitution(
        [
            FindPackageShare('motoman_bringup'),
            'config',
            'controllers.yaml',
        ]
    )

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    controller_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description}, motoman_controllers],
        #remappings=[('joint_states', 'motoman/joint_states')],
        output="screen",
        on_exit=Shutdown(),
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['forward_command_controller_position'],
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    rviz_node = Node(package='rviz2',
			executable='rviz2',
			name='rviz2',
			arguments=['--display-config', rviz_file],
			condition=IfCondition(use_rviz)
	)
    

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            robot_ip_parameter_name,
            description='Hostname or IP address of the robot.')
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            use_rviz_parameter_name,
            default_value='false',
            description='Visualize the robot in Rviz')
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            load_gripper_parameter_name,
            default_value='false',
            description='Use Gripper as an end-effector, otherwise, the robot is loaded '
                        'without an end-effector.')
    )



    gripper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('motoman_gripper'), 'launch', 'gripper.launch.py'])]),
        launch_arguments={robot_ip_parameter_name: robot_ip}.items(),
        condition=IfCondition(load_gripper)
    )

    # ------------------ NODES ------------------
    nodes = [
        controller_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        gripper_launch
    ]

    return LaunchDescription(declared_arguments + nodes)
