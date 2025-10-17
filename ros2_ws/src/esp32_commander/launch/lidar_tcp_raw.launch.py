#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'esp32_ip',
            default_value='192.168.125.222',
            description='ESP32 IP address'
        ),
        
        DeclareLaunchArgument(
            'esp32_port', 
            default_value='3333',
            description='ESP32 TCP port'
        ),
        
        DeclareLaunchArgument(
            'show_raw_data',
            default_value='true',
            description='Show raw LIDAR data in console'
        ),
        
        DeclareLaunchArgument(
            'raw_data_format',
            default_value='both',
            description='Raw data format: hex, ascii, both'
        ),
        
        DeclareLaunchArgument(
            'max_raw_bytes',
            default_value='128',
            description='Maximum bytes to display per raw data chunk'
        ),
        
        DeclareLaunchArgument(
            'raw_data_file',
            default_value='',
            description='File to save raw LIDAR data (leave empty to disable)'
        ),

        # LIDAR TCP Bridge Node
        Node(
            package='esp32_commander',
            executable='lidar_tcp_bridge',
            name='lidar_tcp_bridge',
            output='screen',
            parameters=[{
                'host': LaunchConfiguration('esp32_ip'),
                'port': LaunchConfiguration('esp32_port'),
                'show_raw_data': LaunchConfiguration('show_raw_data'),
                'raw_data_format': LaunchConfiguration('raw_data_format'),
                'max_raw_bytes_per_display': LaunchConfiguration('max_raw_bytes'),
                'raw_data_file': LaunchConfiguration('raw_data_file'),
            }],
            emulate_tty=True,
        )
    ])