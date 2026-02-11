import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'autonomous_litter_bot_package'

    # 1. Get the Path to your URDF
    urdf_file_name = 'robot.urdf' # OR 'go1.urdf.xacro' if you use xacro
    urdf_path = os.path.join(
        get_package_share_directory(pkg_name),
        'urdf',
        urdf_file_name
    )

    # 2. Read the URDF file
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # 3. Robot State Publisher (The "URDF Publisher")
    # This replaces the static transform. It publishes base_link -> laser_frame
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # 4. RF2O (Still Needed for Odom!)
    # This publishes odom -> base_footprint
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom',
            'publish_tf': True, 
            'base_frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 10.0
        }]
    )

    # 5. Slamtec Driver & SLAM Toolbox
    lidar_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('sllidar_ros2'), 'launch', 'sllidar_c1_launch.py')
        )
    )
    
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'params_file': os.path.join(get_package_share_directory(pkg_name), 'config', 'slam_params.yaml'),
            'use_sim_time': 'false'
        }.items()
    )

    rviz_node = Node (
            package='rviz2',
            executable='rviz2',
            name="rviz2",
            output='screen',
        )

    return LaunchDescription([
        rsp_node,       # Publishes static TFs (base -> laser)
        rf2o_node,      # Publishes dynamic TFs (odom -> base)
        lidar_driver,
        slam_toolbox,
        rviz_node
    ])