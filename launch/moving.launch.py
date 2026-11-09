from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'lidar_maze_navigation'

    align_node = Node(
        package=package_name,
        executable='align',
        name='align_node',
        output='screen'
    )

    execute_plan_node = Node(
        package=package_name,
        executable='plan_executor_srv',
        name='plan_executor_srv_node',
        output='screen'
    )

    ransac_node = Node(
        package=package_name,
        executable='ransac_node',
        name='ransac_node',
        output='screen'
    )

    return LaunchDescription([
        align_node,
        execute_plan_node,
        ransac_node
    ])
