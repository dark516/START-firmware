#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lidar_parser',
            executable='lidar_parser_node',
            name='lidar_parser',
            output='screen',
            parameters=[{
                'tcp_ip': '192.168.1.72',
                'tcp_port': 3333,
                'frame_id': 'laser',
                'scan_frequency': 15.0,  # Увеличил частоту
                'range_min': 0.05,
                'range_max': 16.0,
            }]
        )
    ])