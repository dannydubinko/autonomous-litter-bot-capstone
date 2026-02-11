from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Arguments
    vis_arg = DeclareLaunchArgument(
        'enable_vis',
        default_value='true',
        description='Set to "true" to publish debug images, "false" for headless mode.'
    )

    target_ip_arg = DeclareLaunchArgument(
        'target_ip',
        default_value='192.168.12.128', # Change to your receiving device IP
        description='IP address of the device running the receiver'
    )

    # 1. Detector Node
    detector_node = Node(
        package='autonomous_litter_bot_package',
        executable='trash_detection_node',
        name='trash_detector',
        output='screen',
        parameters=[{
            'enable_vis': LaunchConfiguration('enable_vis')
        }]
    )

    # 2. Proto Sender Node
    # sender_node = Node(
    #     package='autonomous_litter_bot_package',
    #     executable='proto_sender_node',
    #     name='proto_sender',
    #     output='screen',
    #     parameters=[{
    #         'target_ip': LaunchConfiguration('target_ip'),
    #         'target_port': 25000
    #     }]
    # )

    return LaunchDescription([
        vis_arg,
        target_ip_arg,
        detector_node,
        # sender_node
    ])