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

        # 1. Locate the existing launch file from the library
    mpu6050_pkg_share = get_package_share_directory('ros2_mpu6050')
    included_mpu_launch = os.path.join(mpu6050_pkg_share, 'launch', 'ros2_mpu6050.launch.py')

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

    
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 
                        'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': os.path.join(
                get_package_share_directory(pkg_name), 
                'config', 'nav2_param.yaml'),
            'use_sim_time': 'false',
            'use_docking_server': 'False',  # <--- ADD THIS LINE
            'use_composition': 'False'      # Also good to keep false for debugging on Pi 5
        }.items()
    )


    map_server = Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': os.path.join(
                    get_package_share_directory(pkg_name), 
                    'maps', 'my_office_map.yaml'),  # path to your saved map
                'use_sim_time': False
            }]
        )


    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            # Use a COLON (:) here, not an EQUALS (=) sign
            'node_names': [
                'map_server', 
                'amcl',
                'planner_server', 
                'controller_server', 
                'behavior_server', 
                'bt_navigator', 
                'waypoint_follower'
            ]
        }]
    )



    return LaunchDescription([
        map_server,
        lifecycle_manager_navigation,
        nav2
    ])

