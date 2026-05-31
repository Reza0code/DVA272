import os
import yaml
from PIL import Image

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


def heuristic(a, b):
    row_a, col_a = a
    row_b, col_b = b

    return abs(row_a - row_b) + abs(col_a - col_b)


def load_map_from_yaml(yaml_path):
    with open(yaml_path, "r") as file:
        map_info = yaml.safe_load(file)

    image_path = os.path.join(
        os.path.dirname(yaml_path),
        map_info["image"]
    )

    resolution = map_info["resolution"]
    origin = map_info["origin"]

    img = Image.open(image_path).convert("L")

    grid = []

    for y in range(img.height):
        row = []

        for x in range(img.width):
            pixel = img.getpixel((x, y))

            # PGM-map:
            # svart = hinder
            # vitt = fri yta
            # grå = okänd yta
            if pixel < 50:
                row.append(1)      # hinder
            elif pixel > 200:
                row.append(0)      # fri yta
            else:
                row.append(1)      # okänd behandlas som hinder

        grid.append(row)

    return grid, resolution, origin


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
    closed_list = []

    parent = {}

    g_score = {}
    g_score[start] = 0

    while open_list:
        current = min(
            open_list,
            key=lambda node: g_score[node] + heuristic(node, goal)
        )

        if current == goal:
            print("Goal reached!")
            return build_path(parent, start, goal)

        open_list.remove(current)
        closed_list.append(current)

        neighbors = get_neighbors(grid, current)

        for neighbor in neighbors:
            if neighbor in closed_list:
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

    return (round(x, 3), round(y, 3))


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


def is_free(grid, point):
    row, col = point

    if row < 0 or row >= len(grid):
        return False

    if col < 0 or col >= len(grid[0]):
        return False

    return grid[row][col] == 0


class AStarPlanner(Node):
    def __init__(self):
        super().__init__("astar_planner")
        self.get_logger().info("A* planner node started")

        self.path_pub = self.create_publisher(Path, "/planned_path", 10)

        yaml_path = "/home/rosdev/projek_ws/maps/first_map/map_1.yaml"

        grid, resolution, origin = load_map_from_yaml(yaml_path)

        origin_x = origin[0]
        origin_y = origin[1]

        self.get_logger().info(
            f"Loaded map: rows={len(grid)}, cols={len(grid[0])}, "
            f"resolution={resolution}, origin=({origin_x}, {origin_y})"
        )

        # Tillfälliga testpunkter i grid-koordinater
        # Vi kan ändra dessa om de hamnar i vägg/okänd yta
        start = (20, 20)
        goal = (45, 45)

        if not is_free(grid, start):
            self.get_logger().error(f"Start is not free: {start}")
            return

        if not is_free(grid, goal):
            self.get_logger().error(f"Goal is not free: {goal}")
            return

        path = astar(grid, start, goal)

        self.get_logger().info(f"Path found: {path}")

        if not path:
            self.get_logger().error("No path found!")
            return

        world_path = []

        for row, col in path:
            world_point = grid_to_world(row, col, resolution, origin_x, origin_y)
            world_path.append(world_point)

        self.get_logger().info(f"World path: {world_path}")

        path_msg = create_path_msg(world_path)
        self.path_pub.publish(path_msg)

        self.get_logger().info("Published path on /planned_path")


def main(args=None):
    rclpy.init(args=args)

    node = AStarPlanner()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()