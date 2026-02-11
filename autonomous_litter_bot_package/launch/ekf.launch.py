from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_name = 'autonomous_litter_bot_package' 
    # Path to the yaml config file
    config_file = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'ekf.yaml'
    )

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[config_file],
            # Optional: Remap topics if your sensors publish to non-standard names
            # remappings=[('/odometry/filtered', '/my_robot/odom_filtered')]
        ),
    ])