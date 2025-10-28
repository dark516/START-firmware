#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'tcp_ip',
            default_value='192.168.1.72',
            description='ESP32 TCP IP address'
        ),
        
        DeclareLaunchArgument(
            'tcp_port',
            default_value='3333',
            description='ESP32 TCP port'
        ),
        
        DeclareLaunchArgument(
            'frame_id',
            default_value='laser',
            description='Frame ID for LaserScan messages'
        ),
        
        DeclareLaunchArgument(
            'scan_frequency',
            default_value='10.0',
            description='Expected scan frequency in Hz'
        ),
        
        DeclareLaunchArgument(
            'range_min',
            default_value='0.05',
            description='Minimum range in meters'
        ),
        
        DeclareLaunchArgument(
            'range_max',
            default_value='16.0',
            description='Maximum range in meters'
        ),

        # Lidar Parser Node
        Node(
            package='lidar_parser',
            executable='lidar_parser_node',
            name='lidar_parser_node',
            output='screen',
            parameters=[{
                'tcp_ip': LaunchConfiguration('tcp_ip'),
                'tcp_port': LaunchConfiguration('tcp_port'),
                'frame_id': LaunchConfiguration('frame_id'),
                'scan_frequency': LaunchConfiguration('scan_frequency'),
                'range_min': LaunchConfiguration('range_min'),
                'range_max': LaunchConfiguration('range_max'),
                'angle_min': -3.14159,  # -π
                'angle_max': 3.14159,   # +π
            }],
            emulate_tty=True,
        )
    ])