#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    esp32_host_arg = DeclareLaunchArgument(
        'esp32_host',
        default_value='192.168.125.241',
        description='ESP32 IP address'
    )
    
    esp32_port_arg = DeclareLaunchArgument(
        'esp32_port', 
        default_value='3333',
        description='ESP32 TCP port'
    )
    
    # Robot physical parameters
    wheel_separation_arg = DeclareLaunchArgument(
        'wheel_separation',
        default_value='0.3',
        description='Distance between wheels in meters'
    )
    
    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.065', 
        description='Wheel radius in meters'
    )
    
    ticks_per_rev_arg = DeclareLaunchArgument(
        'ticks_per_revolution',
        default_value='1440',
        description='Encoder ticks per wheel revolution'
    )

    # ESP32 Commander Node
    esp32_commander_node = Node(
        package='esp32_commander',
        executable='commander_node',
        name='esp32_commander',
        parameters=[{
            'host': LaunchConfiguration('esp32_host'),
            'port': LaunchConfiguration('esp32_port')
        }],
        output='screen'
    )

    # Robot Localization Node  
    robot_localization_node = Node(
        package='esp32_commander',
        executable='robot_localization_node',
        name='robot_localization',
        parameters=[{
            'wheel_separation': LaunchConfiguration('wheel_separation'),
            'wheel_radius': LaunchConfiguration('wheel_radius'),
            'ticks_per_revolution': LaunchConfiguration('ticks_per_revolution')
        }],
        output='screen'
    )

    return LaunchDescription([
        esp32_host_arg,
        esp32_port_arg,
        wheel_separation_arg,
        wheel_radius_arg,
        ticks_per_rev_arg,
        esp32_commander_node,
        robot_localization_node
    ])