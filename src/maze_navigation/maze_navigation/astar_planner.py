import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

def heuristic(a, b):
    row_a, col_a = a
    row_b, col_b = b

    return abs(row_a - row_b) + abs(col_a - col_b)

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

        # Kontrollera att rutan är inne i kartan
        if new_row < 0 or new_row >= len(grid):
            continue

        if new_col < 0 or new_col >= len(grid[0]):
            continue

        # Kontrollera att rutan inte är vägg
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

        #print("Current:", current)
        #print("f(current):", g_score[current] + heuristic(current, goal))
        
        if current == goal:
            print("Goal reached!")
            return build_path(parent, start, goal)

        open_list.remove(current)
        closed_list.append(current)

        #print("open_list:", open_list)
        #print("closed_list:", closed_list)

        neighbors = get_neighbors(grid, current)
        #print("neighbors:", neighbors)

        for neighbor in neighbors:
            if neighbor in closed_list:
                continue

            if neighbor not in open_list:
                open_list.append(neighbor)
                parent[neighbor] = current
                g_score[neighbor] = g_score[current] + 1

        #print("open_list after neighbors:", open_list)
        #print("parent:", parent)
        #print("g_score:", g_score)    

    return []

def print_grid_with_path(grid, path, start, goal):
    display = []

    for row in grid:
        display.append(row.copy())

    for row, col in path:
        display[row][col] = "*"

    start_row, start_col = start
    goal_row, goal_col = goal

    display[start_row][start_col] = "S"
    display[goal_row][goal_col] = "G"

    for row in display:
        print(" ".join(str(cell) for cell in row))

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

class AStarPlanner(Node):
    def __init__(self):
        super().__init__("astar_planner")
        self.get_logger().info("A* planner node started")
        self.path_pub = self.create_publisher(Path, "/planned_path", 10)

        grid = [
            [0, 0, 0, 1, 0, 0],
            [1, 1, 0, 1, 0, 1],
            [0, 0, 0, 0, 0, 1],
            [0, 1, 1, 1, 0, 0],
            [0, 0, 0, 0, 0, 0],
        ]

        start = (0, 0)
        goal = (4, 5)

        path = astar(grid, start, goal)

        self.get_logger().info(f"Path found: {path}")
        resolution = 0.5
        origin_x = 0.0
        origin_y = 0.0

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