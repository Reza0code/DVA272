import os
import math
import yaml
from PIL import Image

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import (
    PoseStamped,
    PointStamped,
    PoseWithCovarianceStamped,
    Point,
)
from visualization_msgs.msg import Marker


def heuristic(a, b):
    row_a, col_a = a
    row_b, col_b = b
    return abs(row_a - row_b) + abs(col_a - col_b)


def load_map_from_yaml(yaml_path):
    with open(yaml_path, "r") as file:
        map_info = yaml.safe_load(file)

    image_path = os.path.join(os.path.dirname(yaml_path), map_info["image"])

    resolution = map_info["resolution"]
    origin = map_info["origin"]

    img = Image.open(image_path).convert("L")

    grid = []

    # ROS map-origin är nere till vänster.
    # PIL läser bilden uppifrån.
    # Därför vänder vi bilden vertikalt.
    for row in range(img.height):
        image_y = img.height - 1 - row
        grid_row = []

        for col in range(img.width):
            pixel = img.getpixel((col, image_y))

            if pixel < 50:
                grid_row.append(1)      # hinder
            elif pixel > 200:
                grid_row.append(0)      # fri yta
            else:
                grid_row.append(1)      # okänd yta behandlas som hinder

        grid.append(grid_row)

    return grid, resolution, origin


def inflate_obstacles(grid, inflation_radius=0):
    inflated = [row.copy() for row in grid]

    rows = len(grid)
    cols = len(grid[0])

    for row in range(rows):
        for col in range(cols):
            if grid[row][col] == 1:
                for dr in range(-inflation_radius, inflation_radius + 1):
                    for dc in range(-inflation_radius, inflation_radius + 1):
                        nr = row + dr
                        nc = col + dc

                        if 0 <= nr < rows and 0 <= nc < cols:
                            inflated[nr][nc] = 1

    return inflated


def is_free(grid, point):
    row, col = point

    if row < 0 or row >= len(grid):
        return False

    if col < 0 or col >= len(grid[0]):
        return False

    return grid[row][col] == 0


def find_nearest_free(grid, point, max_radius=12):
    start_row, start_col = point

    if is_free(grid, point):
        return point

    for radius in range(1, max_radius + 1):
        for dr in range(-radius, radius + 1):
            for dc in range(-radius, radius + 1):
                row = start_row + dr
                col = start_col + dc
                candidate = (row, col)

                if is_free(grid, candidate):
                    return candidate

    return None


def get_neighbors(grid, current):
    row, col = current

    directions = [
        (0, 1),    # höger
        (0, -1),   # vänster
        (1, 0),    # ner
        (-1, 0),   # upp
    ]

    neighbors = []

    for d_row, d_col in directions:
        new_row = row + d_row
        new_col = col + d_col

        if new_row < 0 or new_row >= len(grid):
            continue

        if new_col < 0 or new_col >= len(grid[0]):
            continue

        if grid[new_row][new_col] == 1:
            continue

        neighbors.append((new_row, new_col))

    return neighbors


def build_path(parent, start, goal):
    path = []
    current = goal

    while current != start:
        path.append(current)
        current = parent[current]

    path.append(start)
    path.reverse()

    return path


def astar(grid, start, goal):
    open_list = [start]
    closed_set = set()

    parent = {}
    g_score = {start: 0}

    while open_list:
        current = min(
            open_list,
            key=lambda node: g_score[node] + heuristic(node, goal)
        )

        if current == goal:
            return build_path(parent, start, goal)

        open_list.remove(current)
        closed_set.add(current)

        for neighbor in get_neighbors(grid, current):
            if neighbor in closed_set:
                continue

            tentative_g = g_score[current] + 1

            if neighbor not in open_list:
                open_list.append(neighbor)
                parent[neighbor] = current
                g_score[neighbor] = tentative_g

            elif tentative_g < g_score[neighbor]:
                parent[neighbor] = current
                g_score[neighbor] = tentative_g

    return []


def grid_to_world(row, col, resolution, origin_x, origin_y):
    x = origin_x + (col + 0.5) * resolution
    y = origin_y + (row + 0.5) * resolution

    return round(x, 3), round(y, 3)


def world_to_grid(x, y, resolution, origin_x, origin_y):
    col = int((x - origin_x) / resolution)
    row = int((y - origin_y) / resolution)

    return row, col


def create_path_msg(world_path):
    path_msg = Path()
    path_msg.header.frame_id = "map"

    for x, y in world_path:
        pose = PoseStamped()
        pose.header.frame_id = "map"

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0

        path_msg.poses.append(pose)

    return path_msg


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

    return math.atan2(siny_cosp, cosy_cosp)


class AStarPlanner(Node):
    def __init__(self):
        super().__init__("astar_planner")
        self.get_logger().info("A* planner started: waiting for Publish Point goal")

        self.path_pub = self.create_publisher(Path, "/planned_path", 10)
        self.marker_pub = self.create_publisher(Marker, "/planned_path_marker", 10)

        self.last_path_msg = None
        self.last_marker_msg = None

        # Publicera om path + marker varje sekund så RViz inte missar den
        self.republish_timer = self.create_timer(1.0, self.republish_path)

        # Karta 1
        self.yaml_path = "/home/rosdev/projek_ws/maps/first_map/map_1_copy.yaml"

        # För karta 2 senare:
        # self.yaml_path = "/home/rosdev/projek_ws/maps/second_map/map_2.yaml"

        self.grid, self.resolution, self.origin = load_map_from_yaml(self.yaml_path)

        # Börja med 0 i trång bana.
        # Om A* hittar path stabilt kan vi testa inflation_radius=1 senare.
        self.grid = inflate_obstacles(self.grid, inflation_radius=0)

        self.origin_x = self.origin[0]
        self.origin_y = self.origin[1]

        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None

        self.get_logger().info(
            f"Loaded map: rows={len(self.grid)}, cols={len(self.grid[0])}, "
            f"resolution={self.resolution}, origin=({self.origin_x}, {self.origin_y})"
        )

        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.amcl_callback,
            10
        )

        self.clicked_point_sub = self.create_subscription(
            PointStamped,
            "/clicked_point",
            self.clicked_point_callback,
            10
        )

    def amcl_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    def create_path_marker(self, world_path):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "astar_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        # Linjens tjocklek i RViz
        marker.scale.x = 0.06

        # Grön tydlig linje
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.pose.orientation.w = 1.0

        for x, y in world_path:
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.08
            marker.points.append(p)

        return marker

    def republish_path(self):
        if self.last_path_msg is not None:
            self.last_path_msg.header.stamp = self.get_clock().now().to_msg()

            for pose in self.last_path_msg.poses:
                pose.header.stamp = self.last_path_msg.header.stamp

            self.path_pub.publish(self.last_path_msg)

        if self.last_marker_msg is not None:
            self.last_marker_msg.header.stamp = self.get_clock().now().to_msg()
            self.marker_pub.publish(self.last_marker_msg)

    def clicked_point_callback(self, msg):
        if self.robot_x is None or self.robot_y is None:
            self.get_logger().error(
                "No /amcl_pose received yet. Set 2D Pose Estimate first."
            )
            return

        goal_x = msg.point.x
        goal_y = msg.point.y

        raw_start = world_to_grid(
            self.robot_x,
            self.robot_y,
            self.resolution,
            self.origin_x,
            self.origin_y
        )

        raw_goal = world_to_grid(
            goal_x,
            goal_y,
            self.resolution,
            self.origin_x,
            self.origin_y
        )

        self.get_logger().info(
            f"Robot start world: x={self.robot_x:.3f}, y={self.robot_y:.3f}"
        )
        self.get_logger().info(
            f"Clicked goal world: x={goal_x:.3f}, y={goal_y:.3f}"
        )
        self.get_logger().info(f"Raw start grid: {raw_start}")
        self.get_logger().info(f"Raw goal grid: {raw_goal}")

        start = find_nearest_free(self.grid, raw_start)
        goal = find_nearest_free(self.grid, raw_goal)

        if start is None:
            self.get_logger().error(f"No free start found near {raw_start}")
            return

        if goal is None:
            self.get_logger().error(f"No free goal found near {raw_goal}")
            return

        self.get_logger().info(f"Adjusted start grid: {start}")
        self.get_logger().info(f"Adjusted goal grid: {goal}")

        path = astar(self.grid, start, goal)

        if not path:
            self.get_logger().error("No path found!")
            return

        self.get_logger().info(f"Path found with {len(path)} grid points")

        world_path = []

        for row, col in path:
            x, y = grid_to_world(
                row,
                col,
                self.resolution,
                self.origin_x,
                self.origin_y
            )
            world_path.append((x, y))

        self.get_logger().info(
            f"Start world path: x={world_path[0][0]:.3f}, y={world_path[0][1]:.3f}"
        )
        self.get_logger().info(
            f"Goal world path: x={world_path[-1][0]:.3f}, y={world_path[-1][1]:.3f}"
        )

        path_msg = create_path_msg(world_path)
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for pose in path_msg.poses:
            pose.header.stamp = path_msg.header.stamp

        marker_msg = self.create_path_marker(world_path)

        self.last_path_msg = path_msg
        self.last_marker_msg = marker_msg

        self.path_pub.publish(path_msg)
        self.marker_pub.publish(marker_msg)

        self.get_logger().info(
            "Published path on /planned_path and marker on /planned_path_marker"
        )


def main(args=None):
    rclpy.init(args=args)

    node = AStarPlanner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt caught.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()