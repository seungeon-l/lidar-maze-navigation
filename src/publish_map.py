#!/usr/bin/env python3
import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from lidar_maze_navigation.msg import Cell, Grid, Row
from lidar_maze_navigation.msg import Point, PointArray


class MapPublisher(Node):
    def __init__(self) -> None:
        super().__init__("example_map_publisher")
        self.publisher = self.create_publisher(Grid, "map", 1)
        self.grid = self._build_grid()
        self.timer = self.create_timer(1.0, self._publish_grid)
        self.get_logger().info(f"Publishing map: {self.grid}")

    def _build_grid(self) -> Grid:
        T = Cell.TOP
        B = Cell.BOTTOM
        L = Cell.LEFT
        R = Cell.RIGHT

        package_share = get_package_share_directory("lidar_maze_navigation")
        filename = os.path.join(package_share, "map.txt")
        with open(filename, "r", encoding="utf-8") as map_file:
            map_source = map_file.read()

        namespace = {"T": T, "B": B, "L": L, "R": R}
        exec(f"map_layout = {map_source}", {}, namespace)
        map_layout = namespace["map_layout"]

        grid = Grid()
        for row_layout in map_layout:
            row = Row()
            for cell_layout in row_layout:
                cell = Cell()
                cell.walls = cell_layout
                row.cells.append(cell)
            grid.rows.append(row)

        return grid

    def _publish_grid(self) -> None:
        self.publisher.publish(self.grid)


class GoldPublisher(Node):
    def __init__(self) -> None:
        super().__init__("gold_publisher")
        self.publisher = self.create_publisher(PointArray, "gold", 1)
        self.gold = self._get_gold()
        self.timer = self.create_timer(1.0, self._publish_gold)
        self.get_logger().info(f"Publishing gold locations: {self.gold}")
    
    def _get_gold(self) -> PointArray:

        package_share = get_package_share_directory("lidar_maze_navigation")
        filename = os.path.join(package_share, "gold.txt")
        with open(filename, "r", encoding="utf-8") as gold_file:
            gold_layout = eval(gold_file.read())

        gold_pos = PointArray()
        for row, col in gold_layout:
            point = Point()
            point.row = row
            point.col = col
            gold_pos.points.append(point)

        return gold_pos

    def _publish_gold(self) -> None:
        self.publisher.publish(self.gold)


class PickupPublisher(Node):
    def __init__(self) -> None:
        super().__init__("pickup_publisher")
        self.publisher = self.create_publisher(PointArray, "pickup", 1)
        self.pickup = self._get_pickup()
        self.timer = self.create_timer(1.0, self._publish_pickup)
        self.get_logger().info(f"Publishing pickup locations: {self.pickup}")
    
    def _get_pickup(self) -> PointArray:
        package_share = get_package_share_directory("lidar_maze_navigation")
        filename = os.path.join(package_share, "pickup.txt")
        with open(filename, "r", encoding="utf-8") as pickup_file:
            pickup_layout = eval(pickup_file.read())

        pickup_pos = PointArray()
        
        for row, col in pickup_layout:
            point = Point()
            point.row = row
            point.col = col
            pickup_pos.points.append(point)

        return pickup_pos

    def _publish_pickup(self) -> None:
        self.publisher.publish(self.pickup)


def main() -> None:
    rclpy.init()
    map_node = MapPublisher()
    gold_node = GoldPublisher()
    pickup_node = PickupPublisher()

    executor = MultiThreadedExecutor()
    executor.add_node(map_node)
    executor.add_node(gold_node)
    executor.add_node(pickup_node)

    try:
        executor.spin()
    finally:
        map_node.destroy_node()
        gold_node.destroy_node()
        pickup_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
