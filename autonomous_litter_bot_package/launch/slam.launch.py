import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.actions import GroupAction
from launch_ros.actions import SetRemap

def generate_launch_description():
    pkg_name = 'autonomous_litter_bot_package'

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'params_file': os.path.join(get_package_share_directory(pkg_name), 'config', 'slam_params.yaml'),
            'use_sim_time': 'false'
        }.items()
    )
    delayed_slam = TimerAction(
        period=2.0,
        actions=[slam_toolbox]
    )



    return LaunchDescription([
        delayed_slam
    ])