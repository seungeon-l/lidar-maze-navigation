from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'lidar_maze_navigation'

    # align_node = Node(
    #     package=package_name,
    #     executable='align',
    #     name='align_node',
    #     output='screen'
    # )

    localization_node = Node(
        package=package_name,
        executable='localization',
        name='localization_node',
        output='screen'
    )

    tracking_node = Node(
        package=package_name,
        executable='tracking',
        name='tracking_node',
        output='screen'
    )

    potential_field_node = Node(
        package=package_name,
        executable='potential_field',
        name='potential_field_node',
        output='screen'
    )

    dij_shortest_path_node = Node(
        package=package_name,
        executable='dij_shortest_path',
        name='dij_shortest_path_node',
        output='screen'
    )

    dij_gold_order_node = Node(
        package=package_name,
        executable='dij_gold_order',
        name='dij_gold_order_node',
        output='screen'
    )

    return LaunchDescription([
        # align_node,
        localization_node,
        tracking_node,
        potential_field_node,
        dij_shortest_path_node,
        dij_gold_order_node
    ])
