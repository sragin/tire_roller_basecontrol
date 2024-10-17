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
            package='tire_roller_basecontrol',
            executable='can_parser',
        ),
        Node(
            package='tire_roller_basecontrol',
            executable='can_sender',
        ),
        Node(
            package='tire_roller_basecontrol',
            executable='navigator_base',
        ),
        Node(
            package='tire_roller_basecontrol',
            executable='io_controller',
        ),
        Node(
            package='tire_roller_basecontrol',
            executable='drive_controller',
        ),
        Node(
            package='gps_rclpy_pkg',
            executable='tcpgps_geoid_pub',
            parameters=[
                {'gps_ip': '192.168.110.163'},
                {'gps_port': 11511},
            ]
        ),
    ])
