#!/usr/bin/env python3
#
# Gazebo excitation launch (position-mode OMX + bringup controller yaml + rosbag).
#
# Startup order (aligned with open_manipulator_x_gazebo.launch.py):
#   Gazebo → spawn → spawners → excitation + bag → teardown → Shutdown
#
import os
from datetime import datetime, timezone
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.actions import SetEnvironmentVariable
from launch.actions import Shutdown
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    open_manipulator_description_path = get_package_share_directory(
        'open_manipulator_description'
    )
    open_manipulator_bringup_path = get_package_share_directory(
        'open_manipulator_bringup'
    )

    package_root = Path(__file__).resolve().parents[1]
    default_bag_dir = str(
        package_root
        / 'results'
        / 'bags'
        / f'gazebo_{datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")}'
    )

    world = LaunchConfiguration('world')
    record_bag = LaunchConfiguration('record_bag')
    bag_output = LaunchConfiguration('bag_output')
    excitation_periods = LaunchConfiguration('excitation_periods')

    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(open_manipulator_bringup_path, 'worlds'),
            ':' + str(Path(open_manipulator_description_path).parent.resolve()),
        ],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch'),
            '/gz_sim.launch.py',
        ]),
        launch_arguments=[
            ('gz_args', [world, '.sdf', ' -v 1', ' -r']),
        ],
    )

    xacro_file = os.path.join(
        open_manipulator_description_path,
        'urdf',
        'open_manipulator_x',
        'open_manipulator_x.urdf.xacro',
    )
    doc = xacro.process_file(xacro_file, mappings={'use_sim': 'true'})
    robot_desc = doc.toprettyxml(indent='  ')
    params = {'robot_description': robot_desc, 'use_sim_time': True}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string',
            robot_desc,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0',
            '-name', 'open_manipulator_x',
            '-allow_renaming', 'true',
            '-use_sim', 'true',
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30.0',
            '--switch-timeout', '10.0',
        ],
        output='screen',
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        output='screen',
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller'],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    excitation_runner = Node(
        package='open_manipulator_sysid',
        executable='excitation_runner',
        name='excitation_runner',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'excitation_periods': excitation_periods,
            },
        ],
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

    graceful_teardown = ExecuteProcess(
        cmd=[
            'bash', '-c',
            (
                'pkill -INT -f "ros2 bag record.*joint_states" 2>/dev/null || true; '
                'sleep 2; '
                'pkill -TERM -f "gz sim" 2>/dev/null || true; '
                'sleep 8; '
                'exit 0'
            ),
        ],
        output='screen',
        name='graceful_teardown',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='empty_world',
            description='Gazebo world name (without .sdf)',
        ),
        DeclareLaunchArgument(
            'record_bag',
            default_value='true',
            description='Record /joint_states to rosbag2 during excitation',
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
        DeclareLaunchArgument(
            'sigterm_timeout',
            default_value='30.0',
            description='Launch-wide SIGINT→SIGTERM timeout for ExecuteProcess shutdown',
        ),
        DeclareLaunchArgument(
            'sigkill_timeout',
            default_value='10.0',
            description='Launch-wide SIGTERM→SIGKILL timeout for ExecuteProcess shutdown',
        ),
        gazebo_resource_path,
        gazebo,
        bridge,
        node_robot_state_publisher,
        gz_spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[arm_controller_spawner, gripper_controller_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=arm_controller_spawner,
                on_exit=[excitation_runner, bag_record],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=excitation_runner,
                on_exit=[graceful_teardown],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=graceful_teardown,
                on_exit=[Shutdown(reason='Excitation complete')],
            )
        ),
    ])
