import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('autonomous_litter_bot_package')
    
    # Paths to your specific config files
    nav2_params_path = os.path.join(pkg_share, 'config', 'nav2_param.yaml')
    map_path = os.path.join(pkg_share, 'maps', 'my_office_map.yaml')
    
    # Path to standard Nav2 navigation sub-launch
    nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    return LaunchDescription([
        # --- 1. Map Server ---
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_path},
                        {'use_sim_time': False}]
        ),

        # --- 2. AMCL (Localization) ---
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_params_path]
        ),

        # --- 3. Planner & Controller ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'navigation_launch.py')),
            launch_arguments={
                'params_file': nav2_params_path,
                'use_sim_time': 'False',
                'use_lifecycle_mgr': 'False' # <--- THE FIX: Kills the duplicate manager!
            }.items()
        ),

        # --- 4. Lifecycle Manager (The Master Switch) ---
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                # THE FIX: Removed missing/optional nodes so it doesn't hang
                'node_names': ['map_server', 'amcl', 'controller_server', 'smoother_server', 'planner_server', 'behavior_server', 'bt_navigator', 'waypoint_follower', 'velocity_smoother'],
                'bond_timeout': 15.0,
                'attempt_respawn_reconnection': True,
                'bond_respawn_max_duration': 20.0
            }]
        )
    ])