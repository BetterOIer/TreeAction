#!/usr/bin/env python3
"""
bt_engine.launch.py — BT 决策引擎启动

启动示例:
  ros2 launch r2_bt bt_engine.launch.py                              # 默认 full_match.xml
  ros2 launch r2_bt bt_engine.launch.py tree_file:=meilin_stage.xml   # 美林赛段
  ros2 launch r2_bt bt_engine.launch.py tree_file:=full_match.xml groot2_port:=1668
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ---- 声明启动参数 ----
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
