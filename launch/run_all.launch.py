from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                [get_package_share_directory('ros2_socketcan'),
                    '/launch/socket_can_bridge.launch.xml']
            )
        ),
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                [get_package_share_directory('rosbridge_server'),
                    '/launch/rosbridge_websocket_launch.xml']
            )
        ),
        Node(
            package='wheelloader_control',
            executable='can_parser',
        ),
        Node(
            package='wheelloader_control',
            executable='can_sender',
        ),
        Node(
            package='wheelloader_control',
            executable='navigator',
        ),
        Node(
            package='wheelloader_control',
            executable='io_controller',
        ),
        Node(
            package='wheelloader_control',
            executable='bucket_controller',
        ),
        Node(
            package='wheelloader_control',
            executable='drive_controller',
        ),
    ])
