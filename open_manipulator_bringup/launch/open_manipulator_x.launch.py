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
#这个文件是 Open Manipulator X 的 ROS 2 启动文件，
# 用来把机器人描述、ros2_control 控制器、状态发布器、
# 初始姿态程序和 RViz 按正确顺序启动起来
from launch import LaunchDescription  #ROS 2 launch 文件最终返回的“启动清单”
from launch.actions import DeclareLaunchArgument #用于声明 launch 参数，例如是否启动 RViz、是否使用仿真
from launch.actions import RegisterEventHandler #用于注册事件处理器，比如“某个节点结束后再启动另一个节点”。
from launch.conditions import IfCondition  # 表示 x 为真才启动
from launch.conditions import UnlessCondition #表示 x 为假才启动
from launch.event_handlers import OnProcessExit #用于监听某个进程退出事件
from launch.substitutions import Command #执行命令并把输出作为 launch 参数，例如执行 xacro 生成 URDF
from launch.substitutions import FindExecutable #查找可执行程序路径，例如查找 xacro
from launch.substitutions import LaunchConfiguration #读取 launch 参数的值
from launch.substitutions import PathJoinSubstitution #拼接路径
from launch_ros.actions import Node  #用于定义要启动的 ROS 2 节点
from launch_ros.substitutions import FindPackageShare  #查找 ROS 2 package 的 share 目录路径


def generate_launch_description():
    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'start_rviz', 
            default_value='false', 
            description='Whether to execute rviz2'
        ), # 声明参数 start_rviz，默认不启动 RViz。
        DeclareLaunchArgument(
            'prefix',
            default_value='""',
            description='Prefix of the joint and link names',
        ), # 声明关节和 link 名称前缀，例如多机器人时可以加 robot1_
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Start robot in Gazebo simulation.',
        ), # 是否使用 Gazebo 仿真
        DeclareLaunchArgument(
            'use_mock_hardware',
            default_value='false',
            description='Use mock hardware mirroring command.',
        ),  # 是否使用 mock hardware，也就是假硬件
        DeclareLaunchArgument(
            'mock_sensor_commands',
            default_value='false',
            description='Enable mock sensor commands.',
        ), #是否启用模拟传感器命令
        DeclareLaunchArgument(
            'port_name',
            default_value='/dev/ttyUSB0',
            description='Port name for hardware connection.',
        ), # 真实机械臂连接串口，默认是 /dev/ttyUSB0
        DeclareLaunchArgument(
            'init_position',
            default_value='true',
            description='Whether to launch the init_position node',
        ), #是否启动初始姿态节点
        DeclareLaunchArgument(
            'ros2_control_type',
            default_value='open_manipulator_x_position',
            description='Type of ros2_control',
        ),
        DeclareLaunchArgument(
            'init_position_file',
            default_value='initial_positions.yaml',
            description='Path to the initial position file',
        ),
    ]

    # LaunchConfiguration作用是获取launch文件中声明的参数的值
    #比如参数‘start_rviz’的值是false，则start_rviz的值为false
    start_rviz = LaunchConfiguration('start_rviz')
    prefix = LaunchConfiguration('prefix')
    use_sim = LaunchConfiguration('use_sim')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    mock_sensor_commands = LaunchConfiguration('mock_sensor_commands')
    port_name = LaunchConfiguration('port_name')
    init_position = LaunchConfiguration('init_position')
    ros2_control_type = LaunchConfiguration('ros2_control_type')
    init_position_file = LaunchConfiguration('init_position_file')

    # Generate URDF file using xacro  # 下面的代码相当于执行 一个命令行
    # xacro 文件路径  各种参数   #执行后生成urdf文件
    # 相当于终端运行 xacro open_manipulator_x.urdf.xacro 
    # prefix:="" use_sim:=false 
    # use_mock_hardware:=false 
    # mock_sensor_commands:=false 
    # port_name:=/dev/ttyUSB0 
    # ros2_control_type:=open_manipulator_x_position
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
        'prefix:=',
        prefix,
        ' ',
        'use_sim:=',
        use_sim,
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
        'ros2_control_type:=',
        ros2_control_type,
    ])

    # Paths for configuration files
    controller_manager_config = PathJoinSubstitution([
        FindPackageShare('open_manipulator_bringup'),
        'config',
        'open_manipulator_x',
        'hardware_controller_manager.yaml',
    ])

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('open_manipulator_description'),
        'rviz',
        'open_manipulator.rviz',
    ])

    trajectory_params_file = PathJoinSubstitution([
        FindPackageShare('open_manipulator_bringup'),
        'config',
        'open_manipulator_x',
        init_position_file,
    ])

    # Define nodes 定义一个 ROS 2 节点
    control_node = Node(
        package='controller_manager', #节点来自 controller_manager 包
        executable='ros2_control_node', # 运行的可执行文件是 ros2_control_node
        parameters=[{'robot_description': urdf_file}, controller_manager_config],
        output='both',
        condition=UnlessCondition(use_sim),
    )

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            'gripper_controller',
            'joint_state_broadcaster',
        ],
        output='both',
        parameters=[{'robot_description': urdf_file}],
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': urdf_file, 'use_sim_time': use_sim}],
        output='both',
    )

    joint_trajectory_executor = Node(
        package='open_manipulator_bringup',
        executable='joint_trajectory_executor',
        parameters=[trajectory_params_file],
        output='both',
        condition=IfCondition(init_position),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='both',
        condition=IfCondition(start_rviz),
    )

    # Event handlers to ensure order of execution
    # 设置启动顺序：控制器加载完后再启动 RViz
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner, on_exit=[rviz_node]
        )
    )

    delay_joint_trajectory_executor_after_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[joint_trajectory_executor],
        )
    )

    return LaunchDescription(
        declared_arguments
        + [
            control_node,
            robot_controller_spawner,
            robot_state_publisher_node,
            delay_rviz_after_joint_state_broadcaster_spawner,
            delay_joint_trajectory_executor_after_controllers,
        ]
    )
