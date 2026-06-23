#!/usr/bin/env python3
#
# Hardware excitation launch (position-mode OMX + bringup controller yaml).
#
# Mutually exclusive with Standard control launch on the same port.
# Startup order (aligned with open_manipulator_x.launch.py):
#   ros2_control → spawner → excitation + bag → shutdown
#
from datetime import datetime, timezone
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import RegisterEventHandler
from launch.actions import Shutdown
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_root = Path(__file__).resolve().parents[1]
    default_bag_dir = str(
        package_root
        / 'results'
        / 'bags'
        / f'hardware_{datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")}'
    )

    declared_arguments = [
        DeclareLaunchArgument(
            'port_name',
            default_value='/dev/ttyUSB0',
            description='Dynamixel USB serial port',
        ),
        DeclareLaunchArgument(
            'use_mock_hardware',
            default_value='false',
            description='Use mock_components/GenericSystem instead of Dynamixel hardware',
        ),
        DeclareLaunchArgument(
            'mock_sensor_commands',
            default_value='false',
            description='Mirror commands in mock hardware mode',
        ),
        DeclareLaunchArgument(
            'record_bag',
            default_value='true',
            description='Record /joint_states during excitation',
        ),
        DeclareLaunchArgument(
            'bag_output',
            default_value=default_bag_dir,
            description='Rosbag2 output directory',
        ),
        DeclareLaunchArgument(
            'excitation_periods',
            default_value='1',
            description='Number of Fourier excitation periods after approach/hold',
        ),
    ]

    port_name = LaunchConfiguration('port_name')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    mock_sensor_commands = LaunchConfiguration('mock_sensor_commands')
    record_bag = LaunchConfiguration('record_bag')
    bag_output = LaunchConfiguration('bag_output')
    excitation_periods = LaunchConfiguration('excitation_periods')

    urdf_file = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('open_manipulator_description'),
            'urdf',
            'open_manipulator_x',
            'open_manipulator_x.urdf.xacro',
        ]),
        ' ',
        'prefix:=""',
        ' ',
        'use_sim:=false',
        ' ',
        'use_mock_hardware:=',
        use_mock_hardware,
        ' ',
        'mock_sensor_commands:=',
        mock_sensor_commands,
        ' ',
        'port_name:=',
        port_name,
        ' ',
        'ros2_control_type:=open_manipulator_x_position',
    ])

    controller_manager_config = PathJoinSubstitution([
        FindPackageShare('open_manipulator_bringup'),
        'config',
        'open_manipulator_x',
        'hardware_controller_manager.yaml',
    ])

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': urdf_file}, controller_manager_config],
        output='screen',
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': urdf_file}],
        output='screen',
    )

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            'gripper_controller',
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30.0',
            '--switch-timeout', '10.0',
        ],
        output='screen',
    )

    excitation_runner = Node(
        package='open_manipulator_sysid',
        executable='excitation_runner',
        name='excitation_runner',
        output='screen',
        parameters=[{'excitation_periods': excitation_periods}],
    )

    bag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-o', bag_output,
            '--topics', '/joint_states',
        ],
        output='screen',
        condition=IfCondition(record_bag),
    )

    stop_bag_record = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'pkill -INT -f "ros2 bag record.*joint_states" 2>/dev/null || true; '
            'sleep 2; exit 0',
        ],
        output='screen',
        name='stop_bag_record',
    )

    delay_excitation_after_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[excitation_runner, bag_record],
        )
    )

    delay_bag_stop_after_excitation = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=excitation_runner,
            on_exit=[stop_bag_record],
        )
    )
    
    # 在 bag 记录停止后关闭机械臂
    shutdown_after_bag_stop = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=stop_bag_record,
            on_exit=[Shutdown(reason='Hardware excitation complete')],
        )
    )
    # TODO: 最终力矩使能不关闭 以防止机械臂突然掉电造成危险

    return LaunchDescription(
        declared_arguments
        + [
            control_node,
            robot_state_publisher_node,
            robot_controller_spawner,
            delay_excitation_after_controllers,
            delay_bag_stop_after_excitation,
            shutdown_after_bag_stop,
        ]
    )
