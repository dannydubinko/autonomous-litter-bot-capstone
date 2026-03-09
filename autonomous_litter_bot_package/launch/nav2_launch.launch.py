import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # --- 1. Paths to your files ---
    # Replace 'my_go1_package' with the actual name of your ROS 2 package
    pkg_share = get_package_share_directory('autonomous_litter_bot_package')
    
    # Path to your custom YAML and MAP files
    nav2_params_path = os.path.join(pkg_share, 'config', 'nav2_param.yaml')
    map_path = os.path.join(pkg_share, 'maps', 'my_office_map.yaml')
    
    # Path to the standard Nav2 navigation launch (provided by the OS)
    nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    return LaunchDescription([
        # --- 2. Map Server ---
        # Loads the .yaml map file and publishes it to the /map topic
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_path},
                        {'use_sim_time': False}]
        ),

        # --- 3. AMCL (Localization) ---
        # Uses Lidar + Map to find where the "Rectangle" is
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_params_path]
        ),

        # --- 4. The Navigation Brain (Planner & Controller) ---
        # This starts the MPPI controller and Global Planner
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'navigation_launch.py')),
            launch_arguments={
                'params_file': nav2_params_path,
                'use_sim_time': 'False'
            }.items()
        ),

        # --- 5. Lifecycle Manager (The "Start" Button) ---
        # Nav2 nodes start in an "inactive" state. This node turns them all on.
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': ['map_server', 'amcl', 'controller_server', 
                                'planner_server', 'behavior_server', 'bt_navigator']
            }]
        )
    ])