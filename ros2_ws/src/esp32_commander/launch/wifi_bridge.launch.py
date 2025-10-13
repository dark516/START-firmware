#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    host_arg = DeclareLaunchArgument(
        'host',
        default_value='192.168.125.241',
        description='ESP32 host IP address'
    )
    
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='3333',
        description='ESP32 port number'
    )
    
    # Create the wifi_bridge node
    wifi_bridge_node = Node(
        package='esp32_commander',
        executable='wifi_bridge',
        name='esp32_commander',
        output='screen',
        parameters=[{
            'host': LaunchConfiguration('host'),
            'port': LaunchConfiguration('port')
        }],
        emulate_tty=True,
    )
    
    return LaunchDescription([
        host_arg,
        port_arg,
        wifi_bridge_node,
    ])