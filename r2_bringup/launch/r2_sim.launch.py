#!/usr/bin/env python3
"""
r2_sim.launch.py — 仿真环境启动

启动顺序:
  1. r2_hardware (里程计模拟器 + 各 Action Server)
  2. r2_bt (BT 决策引擎)

启动示例:
  ros2 launch r2_bringup r2_sim.launch.py                                   # 默认 full_match
  ros2 launch r2_bringup r2_sim.launch.py tree_file:=suspension_example.xml  # 悬挂测试树
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ---- BT 引擎启动参数 ----
    tree_file_arg = DeclareLaunchArgument(
        'tree_file',
        default_value='full_match.xml',
        description='行为树 XML 文件名 (位于 r2_bt/trees/ 目录下): '
                    'full_match.xml / meilin_stage.xml / suspension_example.xml')

    groot2_port_arg = DeclareLaunchArgument(
        'groot2_port',
        default_value='1667',
        description='Groot2 监控端口 (0 表示禁用)')

    tick_frequency_arg = DeclareLaunchArgument(
        'tick_frequency',
        default_value='100.0',
        description='行为树 Tick 频率 (Hz)')

    segment_topic_arg = DeclareLaunchArgument(
        'segment_topic',
        default_value='/planning/segments',
        description='接收 Segment Plan 的 ROS2 Topic')

    return LaunchDescription([
        tree_file_arg,
        groot2_port_arg,
        tick_frequency_arg,
        segment_topic_arg,

        # ---- 仿真: 里程计模拟器 ----
        Node(
            package='r2_hardware',
            executable='odom_simulator',
            name='odom_simulator',
            output='screen',
        ),

        # ---- Action Server: 底盘导航 ----
        Node(
            package='r2_hardware',
            executable='move_to_pose_action_server',
            name='move_to_pose_action_server',
            output='screen',
            parameters=[{
                'odom_topic': '/odom_world',
            }],
        ),

        # ---- Action Server: 主动悬挂 ----
        Node(
            package='r2_hardware',
            executable='suspension_action_server',
            name='suspension_action_server',
            output='screen',
        ),

        # ---- Action Server: 机械臂 ----
        Node(
            package='r2_hardware',
            executable='arm_action_server',
            name='arm_action_server',
            output='screen',
        ),

        # ---- Action Server: 矛头机构 ----
        Node(
            package='r2_hardware',
            executable='spear_action_server',
            name='spear_action_server',
            output='screen',
        ),

        # ---- 上层: BT 决策引擎 ----
        Node(
            package='r2_bt',
            executable='r2_bt_engine',
            name='bt_engine_node',
            output='screen',
            parameters=[{
                'tree_file': LaunchConfiguration('tree_file'),
                'groot2_port': LaunchConfiguration('groot2_port'),
                'tick_frequency': LaunchConfiguration('tick_frequency'),
                'segment_topic': LaunchConfiguration('segment_topic'),
            }],
        ),
    ])
