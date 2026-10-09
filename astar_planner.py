"""A* grid planner and ROS 2 node for occupancy-grid navigation."""

from __future__ import annotations

import heapq
import math
from typing import Iterable, Optional


Cell = tuple[int, int]  # (x, y) grid coordinates


def astar_grid(
    width: int,
    height: int,
    start: Cell,
    goal: Cell,
    blocked: Iterable[Cell],
    allow_diagonal: bool = True,
) -> Optional[list[Cell]]:
    """Return a path including start and goal, or None if no path exists.

    Diagonal moves are disallowed when either adjacent orthogonal cell is blocked,
    preventing the path from squeezing through obstacle corners.
    """
    if width <= 0 or height <= 0:
        raise ValueError("width and height must be positive")
    obstacles = set(blocked)

    def in_bounds(cell: Cell) -> bool:
        x, y = cell
        return 0 <= x < width and 0 <= y < height

    if not in_bounds(start) or not in_bounds(goal):
        return None
    if start in obstacles or goal in obstacles:
        return None
    if start == goal:
        return [start]

    cardinal = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0)]
    diagonal = [(1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
                (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2))]
    moves = cardinal + (diagonal if allow_diagonal else [])

    def heuristic(cell: Cell) -> float:
        dx, dy = abs(cell[0] - goal[0]), abs(cell[1] - goal[1])
        if allow_diagonal:
            return max(dx, dy) + (math.sqrt(2) - 1.0) * min(dx, dy)
        return float(dx + dy)

    frontier: list[tuple[float, float, Cell]] = [(heuristic(start), 0.0, start)]
    came_from: dict[Cell, Optional[Cell]] = {start: None}
    cost_so_far: dict[Cell, float] = {start: 0.0}

    while frontier:
        _, current_cost, current = heapq.heappop(frontier)
        if current_cost > cost_so_far.get(current, math.inf):
            continue
        if current == goal:
            path: list[Cell] = []
            cursor: Optional[Cell] = current
            while cursor is not None:
                path.append(cursor)
                cursor = came_from[cursor]
            return list(reversed(path))

        for dx, dy, step_cost in moves:
            neighbor = (current[0] + dx, current[1] + dy)
            if not in_bounds(neighbor) or neighbor in obstacles:
                continue
            if dx and dy:
                if ((current[0] + dx, current[1]) in obstacles or
                        (current[0], current[1] + dy) in obstacles):
                    continue
            new_cost = cost_so_far[current] + step_cost
            if new_cost < cost_so_far.get(neighbor, math.inf):
                cost_so_far[neighbor] = new_cost
                came_from[neighbor] = current
                heapq.heappush(frontier, (new_cost + heuristic(neighbor), new_cost, neighbor))
    return None


try:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import OccupancyGrid, Path
    from rclpy.node import Node

    class AStarPlannerNode(Node):
        """Plan a path on each received goal using the latest occupancy grid."""

        def __init__(self) -> None:
            super().__init__("astar_planner")
            self.declare_parameter("occupied_threshold", 50)
            self.declare_parameter("allow_unknown", False)
            self._map: Optional[OccupancyGrid] = None
            self._map_sub = self.create_subscription(OccupancyGrid, "/map", self._on_map, 10)
            self._goal_sub = self.create_subscription(PoseStamped, "/goal_pose", self._on_goal, 10)
            self._path_pub = self.create_publisher(Path, "/planned_path", 10)

        def _on_map(self, msg: OccupancyGrid) -> None:
            self._map = msg

        def _on_goal(self, goal_msg: PoseStamped) -> None:
            grid = self._map
            if grid is None:
                self.get_logger().warning("No /map received yet; cannot plan")
                return
            if goal_msg.header.frame_id and goal_msg.header.frame_id != grid.header.frame_id:
                self.get_logger().error(
                    f"Goal frame '{goal_msg.header.frame_id}' differs from map frame "
                    f"'{grid.header.frame_id}'. Transform the goal before publishing."
                )
                return
            if grid.info.resolution <= 0 or grid.info.width == 0 or grid.info.height == 0:
                self.get_logger().error("Received an invalid occupancy grid")
                return

            origin = grid.info.origin.position
            resolution = grid.info.resolution

            def world_to_cell(x: float, y: float) -> Cell:
                return (int(math.floor((x - origin.x) / resolution)),
                        int(math.floor((y - origin.y) / resolution)))

            def cell_to_world(cell: Cell) -> tuple[float, float]:
                return (origin.x + (cell[0] + 0.5) * resolution,
                        origin.y + (cell[1] + 0.5) * resolution)

            # A* node currently expects the robot's current position as start. This
            # starter obtains it from a configurable pose topic in a future extension;
            # for now, start is the map origin cell and is deliberately explicit.
            # Prefer setting /start_pose once localization is integrated.
            start_pose = getattr(self, "_start_pose", None)
            if start_pose is None:
                self.get_logger().warning("No start pose available; publish /start_pose to plan")
                return
            start = world_to_cell(start_pose.pose.position.x, start_pose.pose.position.y)
            goal = world_to_cell(goal_msg.pose.position.x, goal_msg.pose.position.y)
            threshold = int(self.get_parameter("occupied_threshold").value)
            allow_unknown = bool(self.get_parameter("allow_unknown").value)
            blocked: set[Cell] = set()
            for y in range(grid.info.height):
                for x in range(grid.info.width):
                    value = grid.data[y * grid.info.width + x]
                    if value >= threshold or (value < 0 and not allow_unknown):
                        blocked.add((x, y))

            cells = astar_grid(grid.info.width, grid.info.height, start, goal, blocked)
            if cells is None:
                self.get_logger().warning(f"No path found from {start} to {goal}")
                return
            path = Path()
            path.header.stamp = self.get_clock().now().to_msg()
            path.header.frame_id = grid.header.frame_id
            for cell in cells:
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x, pose.pose.position.y = cell_to_world(cell)
                pose.pose.position.z = origin.z
                pose.pose.orientation.w = 1.0
                path.poses.append(pose)
            self._path_pub.publish(path)
            self.get_logger().info(f"Published path with {len(path.poses)} poses")

        def _on_start_pose(self, msg: PoseStamped) -> None:
            self._start_pose = msg

    def main(args=None) -> None:
        rclpy.init(args=args)
        node = AStarPlannerNode()
        node._start_sub = node.create_subscription(PoseStamped, "/start_pose", node._on_start_pose, 10)
        try:
            rclpy.spin(node)
        finally:
            node.destroy_node()
            rclpy.shutdown()

except ImportError:
    # Keeps the algorithm usable for standard-Python unit tests without ROS installed.
    def main(args=None) -> None:
        raise RuntimeError("ROS 2 Python packages are unavailable. Source /opt/ros/jazzy/setup.bash.")
