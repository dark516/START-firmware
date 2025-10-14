#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Аргументы
    host_arg = DeclareLaunchArgument(
        'host',
        default_value='10.115.122.247',
        description='ESP32 IP address'
    )
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='3333',
        description='ESP32 port'
    )

    return LaunchDescription([
        host_arg,
        port_arg,

        # ESP32 WiFi Bridge
        Node(
            package='esp32_commander',
            executable='wifi_bridge',
            name='esp32_wifi_bridge',
            output='screen',
            parameters=[{
                'host': LaunchConfiguration('host'),
                'port': LaunchConfiguration('port')
            }],
        ),

        # Узел локализации (ваш собственный executable)
        Node(
            package='esp32_commander',
            executable='robot_localization_node',
            name='robot_localization',
            output='screen',
            parameters=[{
                'wheel_separation': 0.3,
                'wheel_radius': 0.065,
                'ticks_per_revolution': 1440
            }],
            # remappings здесь не нужны, если имена топиков совпадают по умолчанию
        ),

        # Необязательно: сырая колёсная одометрия для сравнения
        # Включайте только если robot_localization_node НЕ публикует TF odom->base_link
        # Node(
        #     package='frob_odometry',
        #     executable='wheel_odometry',  # проверьте имя exec в setup.py
        #     name='wheel_odometry_debug',
        #     output='screen',
        #     parameters=[{
        #         'meters_per_tick': 0.0,        # 0.0 => посчитать из radius и ticks
        #         'wheel_radius': 0.065,
        #         'ticks_per_revolution': 1440,
        #         'wheel_base': 0.3,
        #         'update_rate': 10.0,
        #         'publish_tf': False,            # чтобы не конфликтовать с локализацией
        #         'odom_frame_id': 'odom',
        #         'base_frame_id': 'base_link'
        #     }]
        # ),
    ])