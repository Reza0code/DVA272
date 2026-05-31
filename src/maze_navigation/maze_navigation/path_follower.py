import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

    return math.atan2(siny_cosp, cosy_cosp)


class PathFollower(Node):
    def __init__(self):
        super().__init__("path_follower")
        self.get_logger().info("Path follower node started")

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.path = []
        self.target_index = 0

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        self.front_distance = float("inf")
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

        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            10
        )

        self.timer = self.create_timer(0.1, self.control_loop)

    def path_callback(self, msg):
        self.path = msg.poses
        self.target_index = 0
        self.goal_logged = False

        self.get_logger().info(f"Saved path with {len(self.path)} poses")

        if self.path:
            first_pose = self.path[0].pose.position
            last_pose = self.path[-1].pose.position

            self.get_logger().info(
                f"First point: x={first_pose.x}, y={first_pose.y}"
            )
            self.get_logger().info(
                f"Last point: x={last_pose.x}, y={last_pose.y}"
            )

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    def scan_callback(self, msg):
        front_ranges = []

        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment

        # Framåt ungefär +/- 25 grader
            if -0.45 <= angle <= 0.45:
                if math.isfinite(r):
                    front_ranges.append(r)

        if front_ranges:
            self.front_distance = min(front_ranges)
        else:
            self.front_distance = float("inf")

    def publish_stop(self):
        stop_twist = Twist()
        stop_twist.linear.x = 0.0
        stop_twist.angular.z = 0.0
        self.cmd_pub.publish(stop_twist)

    def control_loop(self):
        if not self.path:
            return

        target_pose = self.path[self.target_index].pose.position

        distance = math.sqrt(
            (target_pose.x - self.robot_x) ** 2 +
            (target_pose.y - self.robot_y) ** 2
        )
        # Om vi är nära sista målet: stoppa och avsluta
        # Stop if final goal is close enough
        if self.target_index == len(self.path) - 1 and distance < 0.50:
            self.publish_stop()

            if not self.goal_logged:
                self.get_logger().info("Final goal reached safely. Stopping robot.")
                self.goal_logged = True

            return

        # Safety stop if obstacle is too close in front
        if self.front_distance < 0.30:
            self.publish_stop()

            self.get_logger().warn(
                f"path blocked by obstacle! front={self.front_distance:.2f}. Robot stopped before goal."
            )
            return

        # Stop only when the robot is actually close to the final goal
        if self.target_index == len(self.path) - 1 and distance < 0.25:
            self.publish_stop()

            if not self.goal_logged:
                self.get_logger().info("Final goal reached. Stopping robot.")
                self.goal_logged = True

            return

        # Move to next target point when close enough
        if distance < 0.25 and self.target_index < len(self.path) - 1:
            self.target_index += 1
            self.get_logger().info(
                f"Moving to next target index: {self.target_index}"
            )
            return

        target_angle = math.atan2(
            target_pose.y - self.robot_y,
            target_pose.x - self.robot_x
        )

        angle_error = target_angle - self.robot_yaw
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        self.get_logger().info(
            f"index={self.target_index}, "
            f"distance={distance:.3f}, "
            f"angle_error={angle_error:.3f}, "
            f"front={self.front_distance:.2f}"
        )

        twist = Twist()

        if abs(angle_error) > 0.6:
            twist.linear.x = 0.0
            twist.angular.z = 0.6 * angle_error
        else:
            twist.linear.x = 0.06
            twist.angular.z = 0.8 * angle_error

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    node = PathFollower()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()