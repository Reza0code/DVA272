import rclpy
import math
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry


class PathFollower(Node):
    def __init__(self):
        super().__init__("path_follower")
        self.get_logger().info("Path follower node started")

        self.path = []
        self.target_index = 0
        self.robot_x = 0.25
        self.robot_y = 0.25
        self.goal_logged = False

        self.path_sub = self.create_subscription(
            Path,
            "/planned_path",
            self.path_callback,
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10
        )
        self.timer = self.create_timer(0.1, self.control_loop)

    def path_callback(self, msg):
        self.path = msg.poses
        self.target_index = 0
        self.goal_logged = False
        self.get_logger().info(f"Saved path with {len(self.path)} poses")

        first_pose = self.path[0].pose.position
        self.get_logger().info(
            f"First point: x={first_pose.x}, y={first_pose.y}"
        )
        last_pose = self.path[-1].pose.position
        self.get_logger().info(
            f"Last point: x={last_pose.x}, y={last_pose.y}"
        )
    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def control_loop(self):
        if not self.path:
            return

        if self.target_index >= len(self.path) - 1:
            if not self.goal_logged:
                self.get_logger().info("Goal reached. Path following complete.")
                self.goal_logged = True
            return

        target_pose = self.path[self.target_index].pose.position
        
        distance = math.sqrt(
            (target_pose.x - self.robot_x) ** 2 +
            (target_pose.y - self.robot_y) ** 2
        )
        if distance < 0.1 and self.target_index < len(self.path) - 1:
            self.target_index += 1
            self.get_logger().info(
                f"Moving to next target index: {self.target_index}"
            )
            return
        

        self.get_logger().info(
            f"Distance to target: {distance:.3f}"
        )

        self.get_logger().info(
            f"Current target index: {self.target_index}, "
            f"x={target_pose.x}, y={target_pose.y}"
        )
        #self.robot_x = target_pose.x
        #self.robot_y = target_pose.y

def main(args=None):
    rclpy.init(args=args)

    node = PathFollower()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()