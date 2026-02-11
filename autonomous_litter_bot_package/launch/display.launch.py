import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Path to your URDF file
    urdf_file = os.path.join(get_package_share_directory('autonomous_litter_bot_package'), 'urdf', 'robot.urdf')

    # 2. Read the file content
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # 3. Start the node
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),

        Node (
            package='rviz2',
            executable='rviz2',
            name="rviz2",
            output='screen',
        ),
    ])