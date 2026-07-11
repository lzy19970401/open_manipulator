#!/usr/bin/env python3
#
# Hardware model-validation trajectory (test profile + rosbag).
#
import os
from datetime import datetime, timezone
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sysid_share = get_package_share_directory('open_manipulator_sysid')
    default_excitation_config = os.path.join(sysid_share, 'config', 'test_trajectory.yaml')

    default_bag_dir = str(
        Path('/workspace')
        / 'sysid_results'
        / 'bags'
        / f'test_hardware_{datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")}'
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
            description='Record excitation bag (/joint_states + /arm_controller/controller_state)',
        ),
        DeclareLaunchArgument(
            'bag_output',
            default_value=default_bag_dir,
            description='Rosbag2 output directory',
        ),
        DeclareLaunchArgument(
            'excitation_periods',
            default_value='2',
            description='Fourier periods for validation (discard_periods=1 in test_trajectory.yaml)',
        ),
        DeclareLaunchArgument(
            'excitation_config',
            default_value=default_excitation_config,
            description='Path to test_trajectory.yaml',
        ),
    ]

    port_name = LaunchConfiguration('port_name')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    mock_sensor_commands = LaunchConfiguration('mock_sensor_commands')
    record_bag = LaunchConfiguration('record_bag')
    bag_output = LaunchConfiguration('bag_output')
    excitation_periods = LaunchConfiguration('excitation_periods')
    excitation_config = LaunchConfiguration('excitation_config')

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('open_manipulator_description'),
        'rviz',
        'open_manipulator.rviz',
    ])

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
        parameters=[
            {
                'excitation_periods': excitation_periods,
                'config': excitation_config,
            },
        ],
    )

    bag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-o', bag_output,
            '--topics', '/joint_states', '/arm_controller/controller_state',
        ],
        output='screen',
        condition=IfCondition(record_bag),
    )

    delay_excitation_after_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[excitation_runner, bag_record],
        )
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        condition=IfCondition(use_mock_hardware),
    )

    return LaunchDescription(
        declared_arguments
        + [
            control_node,
            robot_state_publisher_node,
            robot_controller_spawner,
            delay_excitation_after_controllers,
            rviz_node,
        ]
    )
