from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("go2_robottrack")
    default_config = f"{package_share}/config/go2_robottrack.yaml"
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            DeclareLaunchArgument("mode", default_value="dry-run"),
            Node(
                package="go2_robottrack",
                executable="robottrack_node",
                name="go2_robottrack",
                output="screen",
                parameters=[
                    LaunchConfiguration("config"),
                    {"mode": LaunchConfiguration("mode")},
                ],
            ),
        ]
    )
