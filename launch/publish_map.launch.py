from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="lidar_maze_navigation",
                executable="publish_map",
                name="example_map_publisher",
                output="screen",
            )
        ]
    )
