#! /usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile

from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion


class ObstacleDetection(Node):
    def __init__(self):
        super().__init__("obstacle_detection")
        self.get_logger().info("JAG KÖR DEL 2.2 GO_TO_GOAL + AVOID")

        # Parametrar
        self.declare_parameter("stop_distance", 0.45)
        self.declare_parameter("goal_x", 2.0)
        self.declare_parameter("goal_y", 0.0)

        self.stop_distance = (
            self.get_parameter("stop_distance")
            .get_parameter_value()
            .double_value
        )

        self.goal_x = (
            self.get_parameter("goal_x")
            .get_parameter_value()
            .double_value
        )

        self.goal_y = (
            self.get_parameter("goal_y")
            .get_parameter_value()
            .double_value
        )

        self.get_logger().info(f"Using stop_distance: {self.stop_distance}")
        self.get_logger().info(f"Goal: x={self.goal_x}, y={self.goal_y}")

        # Robot state
        self.pose = Pose()
        self.yaw = 0.0
        self.has_odom_received = False

        # Laser state
        self.scan_ranges = []
        self.angle_min = 0.0
        self.angle_increment = 0.0
        self.range_min = 0.0
        self.range_max = 3.5
        self.has_scan_received = False

        # Control state
        self.goal_reached = False
        self.mode = "GO_TO_GOAL"
        self.avoid_direction = 1.0
        self.avoid_target_yaw = 0.0
        self.avoid_turn_start = None
        self.avoid_forward_start = None

        qos = QoSProfile(depth=10)

        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", qos)

        self.odom_sub = self.create_subscription(
            Odometry,
            "odom",
            self.get_odom_callback,
            qos_profile=qos_profile_sensor_data,
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            "scan",
            self.scan_callback,
            qos_profile=qos_profile_sensor_data,
        )

        self.timer = self.create_timer(0.1, self.timer_callback)

    def get_odom_callback(self, msg):
        self.pose = msg.pose.pose

        q = [
            self.pose.orientation.x,
            self.pose.orientation.y,
            self.pose.orientation.z,
            self.pose.orientation.w,
        ]

        roll, pitch, yaw = euler_from_quaternion(q)
        self.yaw = yaw
        self.has_odom_received = True

    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment
        self.range_min = msg.range_min
        self.range_max = msg.range_max
        self.has_scan_received = True

    def timer_callback(self):
        if self.has_scan_received and self.has_odom_received:
            self.control_robot()

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def get_front_obstacle(self):
        closest_distance = float("inf")
        closest_angle = 0.0

        for i, r in enumerate(self.scan_ranges):
            if math.isinf(r) or math.isnan(r):
                continue

            if r < self.range_min or r > self.range_max:
                continue

            angle = self.angle_min + i * self.angle_increment
            angle = self.normalize_angle(angle)

            # Kolla bara framför roboten, ungefär +-90 grader
            if -0.8 <= angle <= 8.0:
                if r < closest_distance:
                    closest_distance = r
                    closest_angle = angle

        return closest_distance, closest_angle

    def get_sector_distance(self, min_angle, max_angle):
        closest_distance = float("inf")

        for i, r in enumerate(self.scan_ranges):
            if math.isinf(r) or math.isnan(r):
                continue

            if r < self.range_min or r > self.range_max:
                continue

            angle = self.angle_min + i * self.angle_increment
            angle = self.normalize_angle(angle)

            if min_angle <= angle <= max_angle:
                if r < closest_distance:
                    closest_distance = r

        return closest_distance

    def choose_avoid_direction(self, closest_angle):
        # Om hindret är på vänster sida, sväng höger.
        # Om hindret är på höger sida, sväng vänster.
        if closest_angle > 0:
            return -1.0
        else:
            return 1.0

    def control_robot(self):
        twist = Twist()

        if self.goal_reached:
            self.stop_robot()
            return

        robot_x = self.pose.position.x
        robot_y = self.pose.position.y

        dx = self.goal_x - robot_x
        dy = self.goal_y - robot_y

        distance_to_goal = math.sqrt(dx**2 + dy**2)
        goal_angle = math.atan2(dy, dx)
        e_theta_goal = self.normalize_angle(goal_angle - self.yaw)

        self.get_logger().info(
            f"Mode={self.mode}, pos=({robot_x:.2f},{robot_y:.2f}), "
            f"goal_dist={distance_to_goal:.2f}, e_theta={e_theta_goal:.2f}"
        )

        if distance_to_goal < 0.25:
            self.goal_reached = True
            self.stop_robot()
            self.get_logger().info("Goal reached! Robot stopped.")
            return

        closest_distance, closest_angle = self.get_front_obstacle()

        # -------------------------
        # GO TO GOAL
        # -------------------------
        if self.mode == "GO_TO_GOAL":
            if closest_distance < self.stop_distance:
                self.avoid_direction = self.choose_avoid_direction(closest_angle)

                self.avoid_target_yaw = self.normalize_angle(
                    self.yaw + self.avoid_direction * math.pi / 3
                )

                self.mode = "AVOID_TURN"
                self.avoid_turn_start = self.get_clock().now()

                self.get_logger().info(
                    f"Obstacle detected: {closest_distance:.2f} m. "
                    f"Switch to AVOID_TURN."
                )

                self.stop_robot()
                return

            P = 0.6
            twist.angular.z = P * e_theta_goal
            twist.angular.z = max(min(twist.angular.z, 0.35), -0.35)

            # Om vinkelfelet är stort: stå still och rotera först
            if abs(e_theta_goal) > 0.85:
                twist.linear.x = 0.0
            elif abs(e_theta_goal) > 0.45:
                twist.linear.x = 0.04
            else:
                twist.linear.x = 0.10

            self.cmd_vel_pub.publish(twist)
            return

        # -------------------------
        # AVOID TURN
        # -------------------------
        if self.mode == "AVOID_TURN":
            e_theta_avoid = self.normalize_angle(self.avoid_target_yaw - self.yaw)

            twist.linear.x = 0.0
            twist.angular.z = 0.9 * e_theta_avoid
            twist.angular.z = max(min(twist.angular.z, 0.5), -0.5)

            now = self.get_clock().now()
            elapsed = (now - self.avoid_turn_start).nanoseconds / 1e9

            if abs(e_theta_avoid) < 0.15 or elapsed > 2.0:
                self.mode = "AVOID_FORWARD"
                self.avoid_forward_start = self.get_clock().now()
                twist.angular.z = 0.0

                self.get_logger().info("Finished turning. Switch to AVOID_FORWARD.")

            self.cmd_vel_pub.publish(twist)
            return

        # -------------------------
        # AVOID FORWARD
        # -------------------------
        if self.mode == "AVOID_FORWARD":
            now = self.get_clock().now()
            elapsed = (now - self.avoid_forward_start).nanoseconds / 1e9

            front_clearance = self.get_sector_distance(-0.35, 0.35)

            if front_clearance < self.stop_distance:
                self.mode = "AVOID_TURN"
                self.avoid_target_yaw = self.normalize_angle(
                    self.yaw + self.avoid_direction * math.pi / 4
                )
                self.avoid_turn_start = self.get_clock().now()
                self.stop_robot()
                return

            twist.linear.x = 0.08
            twist.angular.z = 0.05 * e_theta_goal
            twist.angular.z = max(min(twist.angular.z, 0.15), -0.15)

            if elapsed > 2.0:
                self.mode = "GO_TO_GOAL"
                self.get_logger().info("Back to GO_TO_GOAL.")

            self.cmd_vel_pub.publish(twist)
            return

    def destroy_node(self):
        self.get_logger().info("Shutting down, stopping robot...")
        self.stop_robot()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = ObstacleDetection()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt caught.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()