import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import GroupAction
from launch_ros.actions import SetRemap
from launch.actions import TimerAction

def generate_launch_description():
    pkg_name = 'autonomous_litter_bot_package'
    # 1. Get the Path to your URDF
    urdf_file_name = 'robot.urdf' # OR 'go1.urdf.xacro' if you use xacro
    urdf_path = os.path.join(
        get_package_share_directory(pkg_name),
        'urdf',
        urdf_file_name
    )
    ekf_config_file = os.path.join(get_package_share_directory(pkg_name), 'config', 'ekf.yaml')

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
            'odom_topic': '/odom_rf2o',
            'publish_tf': False, 
            'base_frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 20.0
        }]
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file],
        remappings=[('odometry/filtered', '/odom')] # Optional: Keeps standard topic name
    )

    # 5. Slamtec Driver & SLAM Toolbox
    lidar_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('sllidar_ros2'), 'launch', 'sllidar_c1_launch.py')
        ),
        launch_arguments={
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': '460800',
            'frame_id': 'laser',          # Match this to your URDF link name
            'inverted': 'true',           # Fixes the upside-down orientation
            'angle_compensate': 'true',
            'scan_mode': 'Standard'
        }.items()
    )
    lidar_remap = GroupAction(
        actions=[SetRemap(src='/scan',dst='/scan_raw'),
        lidar_driver
        ]
    )

    lidar_filter = Node(
        package='autonomous_litter_bot_package',
        executable='lidar_filter_node',
        name='lidar_filter',
        output='screen',
    ) 

    imu_6050 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros2_mpu6050'), 'launch', 'ros2_mpu6050.launch.py')
        ),
    )

    imu_madwick = Node(
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


    return LaunchDescription([
        rsp_node,       
        rf2o_node,      
        lidar_remap,
        ekf_node,
        imu_6050,
        imu_madwick,
        lidar_filter,
    ])

