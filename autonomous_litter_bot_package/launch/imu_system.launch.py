import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    # 1. Locate the existing launch file from the library
    mpu6050_pkg_share = get_package_share_directory('ros2_mpu6050')
    included_mpu_launch = os.path.join(mpu6050_pkg_share, 'launch', 'ros2_mpu6050.launch.py')

    return LaunchDescription([
        
        # ---------------------------------------------------------
        # PART 1: Include the MPU6050 Driver (The "Nested" Launch)
        # ---------------------------------------------------------
        # This effectively runs "ros2 launch ros2_mpu6050 ros2_mpu6050.launch.py"
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(included_mpu_launch)
        ),

        # ---------------------------------------------------------
        # PART 2: The Madgwick Filter (Still a separate Node)
        # ---------------------------------------------------------
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            output='screen',
            parameters=[{
                'use_mag': False,
                'publish_tf': False,
                'world_frame': 'enu'
            }],
            remappings=[
                ('/imu/data_raw', '/imu/mpu6050')
            ]
        )
    ])