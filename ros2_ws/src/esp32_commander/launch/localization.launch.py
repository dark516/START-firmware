#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Аргументы
    esp32_ip_arg = DeclareLaunchArgument(
        'esp32_ip',
        default_value='192.168.125.222',
        description='ESP32 IP address'
    )
    cmd_port_arg = DeclareLaunchArgument(
        'cmd_port',
        default_value='3333',
        description='ESP32 UDP command port'
    )
    sensor_port_arg = DeclareLaunchArgument(
        'sensor_port',
        default_value='3335',
        description='ESP32 UDP sensor data port'
    )
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value='3334',
        description='ESP32 UDP lidar port'
    )

    return LaunchDescription([
        esp32_ip_arg,
        cmd_port_arg,
        sensor_port_arg,
        lidar_port_arg,

        # ============================================
        # 1. ESP32 UDP Bridge (команды и данные датчиков)
        # ============================================
        Node(
            package='esp32_commander',
            executable='wifi_bridge',
            name='esp32_wifi_bridge',
            output='screen',
            parameters=[{
                'esp32_ip': LaunchConfiguration('esp32_ip'),
                'cmd_port': LaunchConfiguration('cmd_port'),
                'sensor_port': LaunchConfiguration('sensor_port')
            }],
        ),

        # ============================================
        # 2. Robot Localization (одометрия + IMU)
        # ============================================
        Node(
            package='esp32_commander',
            executable='robot_localization_node',
            name='robot_localization',
            output='screen',
            parameters=[{
                'wheel_separation': 0.3,
                'wheel_radius': 0.065,
                'ticks_per_revolution': 1440,
                'publish_rate': 20.0
            }],
        ),

        # ============================================
        # 3. RPLidar UDP Bridge (лидар -> /scan)
        # ============================================
        Node(
            package='esp32_commander',
            executable='lidar_bridge',
            name='rplidar_bridge',
            output='screen',
            parameters=[{
                'esp32_ip': LaunchConfiguration('esp32_ip'),
                'lidar_port': LaunchConfiguration('lidar_port'),
                'cmd_port': LaunchConfiguration('cmd_port'),
                'frame_id': 'laser',
                'range_min': 0.15,
                'range_max': 12.0,
                'scan_frequency': 10.0
            }],
        ),

        # ============================================
        # 4. Static Transform: base_link -> laser
        # ============================================
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser'],
            # Аргументы: x y z yaw pitch roll parent_frame child_frame
            # Подстрой координаты под свой робот!
        ),
    ])