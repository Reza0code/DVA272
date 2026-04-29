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
        self.get_logger().info("JAG KÖR DEL 2.3 MJUKARE VÄJNING MED VIKTNING")

        # -----------------------------
        # Parametrar
        # -----------------------------
        self.declare_parameter("stop_distance", 0.30)
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

        # -----------------------------
        # Robot state
        # -----------------------------
        self.pose = Pose()
        self.yaw = 0.0
        self.has_odom_received = False

        # -----------------------------
        # Laser state
        # -----------------------------
        self.scan_ranges = []
        self.angle_min = 0.0
        self.angle_increment = 0.0
        self.range_min = 0.0
        self.range_max = 3.5
        self.has_scan_received = False

        # -----------------------------
        # Control state
        # -----------------------------
        self.goal_reached = False
        self.avoid_direction = 1.0

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

    # --------------------------------------------------
    # Odometry
    # --------------------------------------------------
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

    # --------------------------------------------------
    # LaserScan
    # --------------------------------------------------
    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment
        self.range_min = msg.range_min
        self.range_max = msg.range_max
        self.has_scan_received = True

    # --------------------------------------------------
    # Timer
    # --------------------------------------------------
    def timer_callback(self):
        if self.has_scan_received and self.has_odom_received:
            self.control_robot()

    # --------------------------------------------------
    # Hjälpfunktioner
    # --------------------------------------------------
    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    # --------------------------------------------------
    # Hitta närmaste hinder framför roboten
    # --------------------------------------------------
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

            # Framför roboten: ungefär +-45 grader
            # Viktigt: normalisering gör att höger sida inte blir blind.
            if -0.8 <= angle <= 0.8:
                if r < closest_distance:
                    closest_distance = r
                    closest_angle = angle

        return closest_distance, closest_angle

    # --------------------------------------------------
    # Hitta närmaste hinder i en sektor
    # --------------------------------------------------
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

    # --------------------------------------------------
    # Välj väjningsriktning
    # --------------------------------------------------
    def choose_avoid_direction(self, closest_angle):
        if closest_angle > 0.15:
            self.last_avoid_direction = -1.0   # hinder vänster → sväng höger
        elif closest_angle < -0.15:
            self.last_avoid_direction = 1.0    # hinder höger → sväng vänster

        return self.last_avoid_direction
    # --------------------------------------------------
    # Huvudlogik: Del 2.3 viktad styrning
    # --------------------------------------------------
    def control_robot(self):
        twist = Twist()

        if self.goal_reached:
            self.stop_robot()
            return

        # -----------------------------
        # Position och mål
        # -----------------------------
        spawn_x = -1.5
        spawn_y = -0.4

        robot_x = self.pose.position.x + spawn_x
        robot_y = self.pose.position.y + spawn_y
        dx = self.goal_x - robot_x
        dy = self.goal_y - robot_y

        distance_to_goal = math.sqrt(dx**2 + dy**2)
        goal_angle = math.atan2(dy, dx)
        e_theta_goal = self.normalize_angle(goal_angle - self.yaw)

        # -----------------------------
        # Mål nått
        # -----------------------------
        if distance_to_goal < 0.25:
            self.goal_reached = True
            self.stop_robot()
            self.get_logger().info("Goal reached! Robot stopped.")
            return

        # -----------------------------
        # Hinder framför roboten
        # -----------------------------
        closest_distance, closest_angle = self.get_front_obstacle()

        # -----------------------------
        # Del 2.3: viktning
        # -----------------------------
        avoid_start_distance = self.stop_distance + 0.4

        if closest_distance == float("inf"):
            avoid_weight = 0.0
        elif closest_distance <= self.stop_distance:
            avoid_weight = 1.0
        elif closest_distance >= avoid_start_distance:
            avoid_weight = 0.0
        else:
            avoid_weight = (avoid_start_distance - closest_distance) / (
                avoid_start_distance - self.stop_distance
            )

        goal_weight = 1.0 - avoid_weight

        # -----------------------------
        # Riktning bort från hinder
        # -----------------------------
        if avoid_weight > 0.0:
            self.avoid_direction = self.choose_avoid_direction(closest_angle)

            avoid_angle = self.normalize_angle(
                self.yaw + self.avoid_direction * math.pi / 3
            )

            e_theta_avoid = self.normalize_angle(avoid_angle - self.yaw)
        else:
            e_theta_avoid = 0.0

        # -----------------------------
        # Kombinera mål och väjning
        # -----------------------------
        combined_error = (
            goal_weight * e_theta_goal
            + avoid_weight * e_theta_avoid
        )

        combined_error = self.normalize_angle(combined_error)

        # -----------------------------
        # P-regulator
        # -----------------------------
        P = 0.6
        twist.angular.z = P * combined_error
        twist.angular.z = max(min(twist.angular.z, 0.35), -0.35)

        # -----------------------------
        # Framåthastighet
        # -----------------------------
        if abs(combined_error) > 0.85:
            twist.linear.x = 0.0
        elif abs(combined_error) > 0.45:
            twist.linear.x = 0.04
        else:
            if avoid_weight > 0.6:
                twist.linear.x = 0.035
            elif avoid_weight > 0.3:
                twist.linear.x = 0.06
            else:
                twist.linear.x = 0.10

        self.get_logger().info(
            f"2.3 weights: goal={goal_weight:.2f}, "
            f"avoid={avoid_weight:.2f}, "
            f"combined_error={combined_error:.2f}, "
            f"obstacle={closest_distance:.2f}, "
            f"goal_dist={distance_to_goal:.2f}"
        )

        self.cmd_vel_pub.publish(twist)

    # --------------------------------------------------
    # Stoppa roboten vid avstängning
    # --------------------------------------------------
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