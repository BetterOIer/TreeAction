#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
meilin_translator.launch.py - 梅林翻译器启动文件

启动示例:
  ros2 launch r2_planner meilin_translator.launch.py
  ros2 launch r2_planner meilin_translator.launch.py is_red_zone:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 启动参数
    is_red_zone_arg = DeclareLaunchArgument(
        'is_red_zone',
        default_value='false',
        description='是否为红区 (红区 row=2 高度分布不同)'
    )

    grid_size_arg = DeclareLaunchArgument(
        'grid_size',
        default_value='1.2',
        description='网格尺寸 (m)'
    )

    grasp_distance_arg = DeclareLaunchArgument(
        'grasp_distance',
        default_value='0.4',
        description='抓取时车身距离格子边线的距离 (m)'
    )

    max_body_capacity_arg = DeclareLaunchArgument(
        'max_body_capacity',
        default_value='3',
        description='车体最大存储 KFS 数量'
    )

    return LaunchDescription([
        is_red_zone_arg,
        grid_size_arg,
        grasp_distance_arg,
        max_body_capacity_arg,

        # 梅林翻译器节点
        Node(
            package='r2_planner',
            executable='meilin_translator',
            name='meilin_translator',
            output='screen',
            parameters=[{
                'grid_size': LaunchConfiguration('grid_size'),
                'grid_origin': [1.2, 1.2],  # (0,0) 格子中心坐标
                'grasp_distance': LaunchConfiguration('grasp_distance'),
                'max_body_capacity': LaunchConfiguration('max_body_capacity'),
                'is_red_zone': LaunchConfiguration('is_red_zone'),
                'input_topic': '/mf_action_seq',
                'output_topic': '/planning/segments',
            }],
        ),
    ])