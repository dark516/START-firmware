#!/usr/bin/env python3
"""
Главный launch-файл для робота Frob
Запускает все необходимые компоненты в правильном порядке
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # ========== АРГУМЕНТЫ ==========
    
    esp32_ip_arg = DeclareLaunchArgument(
        'esp32_ip',
        default_value='10.93.129.246'   ,
        description='IP address of ESP32'
    )
    
    use_slam_arg = DeclareLaunchArgument(
        'use_slam',
        default_value='false',
        description='Launch SLAM for mapping'
    )
    
    use_nav_arg = DeclareLaunchArgument(
        'use_nav',
        default_value='false',
        description='Launch Nav2 navigation (requires existing map)'
    )
    
    use_display_arg = DeclareLaunchArgument(
        'use_display',
        default_value='false',
        description='Launch display.launch for URDF visualization'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch standalone RViz2'
    )
    
    # ========== КОНФИГУРАЦИЯ ==========
    
    esp32_ip = LaunchConfiguration('esp32_ip')
    use_slam = LaunchConfiguration('use_slam')
    use_nav = LaunchConfiguration('use_nav')
    use_display = LaunchConfiguration('use_display')
    use_rviz = LaunchConfiguration('use_rviz')
    
    # ========== 1. БАЗОВЫЕ НОДЫ РОБОТА (ВСЕГДА ЗАПУСКАЮТСЯ) ==========
    
    # 1.1 WiFi Bridge (ESP32 → ROS2)
    wifi_bridge = Node(
        package='esp32_commander',
        executable='wifi_bridge_upd',
        name='wifi_bridge',
        output='screen',
        parameters=[{
            'esp32_ip': esp32_ip,
            'cmd_port': 3333,
            'sensor_port': 3335,
            'lidar_port': 3334
        }],
        respawn=True,
        respawn_delay=2.0
    )
    
    # 1.2 RPLidar Parser (сырые байты → /scan)
    lidar_parser = Node(
        package='lidar_parser',
        executable='rplidar_tcp_node_upd',  # ← твоя нода!
        name='rplidar_parser',
        output='screen',
        parameters=[{
            'frame_id': 'laser',
            'range_min': 0.15,
            'range_max': 8.0,
            'enable_sor_filter': True,
            'sor_k_neighbors': 8,
            'sor_std_dev_multiplier': 1.0
        }],
        respawn=True,
        respawn_delay=2.0
    )
    
    # 1.3 Static TF: base_link → laser
    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser'],
        output='screen'
    )
    
    # 1.4 Robot Localization (EKF фильтр: энкодеры + IMU → /odom)
    # Запускаем с небольшой задержкой, чтобы wifi_bridge успел запуститься
    robot_localization = TimerAction(
        period=2.0,  # Задержка 2 секунды
        actions=[
            Node(
                package='esp32_commander',
                executable='robot_localization_node',
                name='robot_localization',
                output='screen',
                parameters=[{
                    'use_sim_time': False
                }],
                respawn=True,
                respawn_delay=2.0
            )
        ]
    )
    
    # ========== 2. URDF / DISPLAY (ОПЦИОНАЛЬНО) ==========
    
    # Вариант A: Используем display.launch (если use_display:=true)
    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robot_description'),
                'launch',
                'display.launch.py'
            )
        ),
        condition=IfCondition(use_display)
    )
    
    # Вариант B: Standalone RViz (если use_rviz:=true и НЕ use_display)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz)
    )
    
    # ========== 3. SLAM (ОПЦИОНАЛЬНО) ==========
    
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_node'),
                'launch',
                'slam.launch.py'
            )
        ),
        condition=IfCondition(use_slam)
    )
    
    # ========== 4. НАВИГАЦИЯ (ОПЦИОНАЛЬНО) ==========
    
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('frob_navigation'),
                'launch',
                'navigation.launch.py'
            )
        ),
        condition=IfCondition(use_nav)
    )
    
    # ========== LAUNCH DESCRIPTION ==========
    
    return LaunchDescription([
        # Аргументы
        esp32_ip_arg,
        use_slam_arg,
        use_nav_arg,
        use_display_arg,
        use_rviz_arg,
        
        # Базовые ноды (всегда запускаются)
        wifi_bridge,
        lidar_parser,
        static_tf_laser,
        robot_localization,
        
        # Опциональные компоненты
        display_launch,
        rviz_node,
        slam_launch,
        navigation_launch,
    ])