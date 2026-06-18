#!/usr/bin/env python3
#
# Copyright 2024 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Wonho Yun, Sungho Woo, Woojin Wie

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.actions import SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # Launch Arguments
    open_manipulator_description_path = os.path.join(
        get_package_share_directory('open_manipulator_description')
    )

    open_manipulator_bringup_path = os.path.join(
        get_package_share_directory('open_manipulator_bringup')
    )

    # Set gazebo sim resource path
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(open_manipulator_bringup_path, 'worlds'),
            ':' + str(Path(open_manipulator_description_path).parent.resolve()),
        ],
    )

    arguments = LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='empty_world', description='Gz sim World'
        ),
    ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch'),
            '/gz_sim.launch.py',
        ]),
        launch_arguments=[
            ('gz_args', [LaunchConfiguration('world'), '.sdf', ' -v 1', ' -r'])
        ],
    )

    xacro_file = os.path.join(
        open_manipulator_description_path,
        'urdf',
        'open_manipulator_x',
        'open_manipulator_x.urdf.xacro',
    )

    #等价于终端执行xacro open_manipulator_x.urdf.xacro use_sim:=true
    doc = xacro.process_file(xacro_file, mappings={'use_sim': 'true'})

    #将doc转换为xml格式 缩进为两个空格便于阅读
    robot_desc = doc.toprettyxml(indent='  ')

    #robot_desc 是最终的robot_description
    params = {'robot_description': robot_desc}

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
            '-x',
            '0.0',
            '-y',
            '0.0',
            '-z',
            '0.0',
            '-R',
            '0.0',
            '-P',
            '0.0',
            '-Y',
            '0.0',
            '-name',
            'open_manipulator_x',
            '-allow_renaming',
            'true',
            '-use_sim',
            'true',
        ],
    )

    # Controller spawner nodes
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout', '30.0', # 控制器管理器超时时间 30秒
            '--switch-timeout', '10.0', # 控制器切换超时时间 10秒
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

    # rviz_config_file = os.path.join(
    #     open_manipulator_description_path, 'rviz', 'open_manipulator.rviz'
    # )

    # rviz = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='log',
    #     arguments=['-d', rviz_config_file],
    # )

    return LaunchDescription([
        # 在gz_spawn_entity 之后 启动 joint_state_broadcaster_spawner
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,   #
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[arm_controller_spawner, gripper_controller_spawner],
            )
        ),
        bridge,
        gazebo_resource_path,
        arguments,
        gazebo,
        node_robot_state_publisher,
        gz_spawn_entity,
        # rviz,
    ])
