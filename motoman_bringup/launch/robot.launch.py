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


def generate_launch_description():
    robot_ip_parameter_name = 'robot_ip'
    load_gripper_parameter_name = 'load_gripper'
    # use_fake_hardware_parameter_name = 'use_fake_hardware'
    # fake_sensor_commands_parameter_name = 'fake_sensor_commands'
    use_rviz_parameter_name = 'use_rviz'

    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    load_gripper = LaunchConfiguration(load_gripper_parameter_name)
    # use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    # fake_sensor_commands = LaunchConfiguration(fake_sensor_commands_parameter_name)
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

    return LaunchDescription([
        DeclareLaunchArgument(
            robot_ip_parameter_name,
            description='Hostname or IP address of the robot.'),
        DeclareLaunchArgument(
            use_rviz_parameter_name,
            default_value='false',
            description='Visualize the robot in Rviz'),
        # DeclareLaunchArgument(
        #     use_fake_hardware_parameter_name,
        #     default_value='false',
        #     description='Use fake hardware'),
        # DeclareLaunchArgument(
        #     fake_sensor_commands_parameter_name,
        #     default_value='false',
        #     description="Fake sensor commands. Only valid when '{}' is true".format(
        #         use_fake_hardware_parameter_name)),
        DeclareLaunchArgument(
            load_gripper_parameter_name,
            default_value='false',
            description='Use Gripper as an end-effector, otherwise, the robot is loaded '
                        'without an end-effector.'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        # Node(
        #     package='joint_state_publisher',
        #     executable='joint_state_publisher',
        #     name='joint_state_publisher',
        #     parameters=[
        #         {'source_list': ['motoman/joint_states'],#'gripper/joint_states'],
        #          'rate': 30}],
        # ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[{'robot_description': robot_description}, motoman_controllers],
            #remappings=[('joint_states', 'motoman/joint_states')],
            output="screen",
            on_exit=Shutdown(),
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution(
                [FindPackageShare('motoman_gripper'), 'launch', 'gripper.launch.py'])]),
            launch_arguments={robot_ip_parameter_name: robot_ip,
                              #use_fake_hardware_parameter_name: use_fake_hardware
                              }.items(),
            condition=IfCondition(load_gripper)

        ),

        Node(package='rviz2',
			executable='rviz2',
			name='rviz2',
			arguments=['--display-config', rviz_file],
			condition=IfCondition(use_rviz)
		)

    ])
