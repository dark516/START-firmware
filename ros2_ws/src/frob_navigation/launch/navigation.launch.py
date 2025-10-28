import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # Пути к файлам
    pkg_dir = get_package_share_directory('frob_navigation')
    nav2_params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    map_yaml_file = os.path.join(pkg_dir, 'maps', 'map.yaml')
    
    # Launch аргументы
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    map_file = LaunchConfiguration('map')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='True',
        description='Automatically startup the nav2 stack')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params_file,
        description='Full path to the ROS2 parameters file to use')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=map_yaml_file,
        description='Full path to map yaml file to load')

    # Создаём перезаписанные параметры (добавляем use_sim_time автоматически)
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_file,
        'autostart': autostart
    }

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True)

    # Ноды Nav2
    return LaunchDescription([
        # Аргументы
        declare_use_sim_time_cmd,
        declare_autostart_cmd,
        declare_params_file_cmd,
        declare_map_yaml_cmd,

        # Группа нод
        GroupAction([
            # Map Server
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[configured_params]),

            # AMCL
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                parameters=[configured_params]),

            # Controller Server
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                parameters=[configured_params]),

            # Smoother Server
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                parameters=[configured_params]),

            # Planner Server
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[configured_params]),

            # Behavior Server
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                parameters=[configured_params]),

            # BT Navigator
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[configured_params]),

            # Waypoint Follower
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                parameters=[configured_params]),

            # Velocity Smoother
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                parameters=[configured_params]),

            # Collision Monitor
            Node(
                package='nav2_collision_monitor',
                executable='collision_monitor',
                name='collision_monitor',
                output='screen',
                parameters=[configured_params]),

            # Lifecycle Manager для локализации
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time},
                           {'autostart': autostart},
                           {'node_names': ['map_server', 'amcl']}]),

            # Lifecycle Manager для навигации
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time},
                           {'autostart': autostart},
                           {'node_names': [
                               'controller_server',
                               'smoother_server',
                               'planner_server',
                               'behavior_server',
                               'bt_navigator',
                               'waypoint_follower',
                               'velocity_smoother',
                               'collision_monitor'
                           ]}]),
        ]),
    ])