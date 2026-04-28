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
        self.get_logger().info("JAG KÖR obstacle_detection_working.py")

        # -----------------------------
        # Parametrar
        # -----------------------------
        self.declare_parameter("stop_distance", 0.45)
        self.declare_parameter("goal_x", 2.0)
        self.declare_parameter("goal_y", 1.0)

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

        self.get_logger().info(f"Using stop_distance: {self.stop_distance} m")
        self.get_logger().info(f"Goal: x={self.goal_x}, y={self.goal_y}")

        # -----------------------------
        # Robotens tillstånd
        # -----------------------------
        self.pose = Pose()
        self.yaw = 0.0

        # False = mål anges i /odom-koordinater
        # True = mål anges i Gazebo world-koordinater med spawn offset
        self.use_spawn_offset = False
        self.spawn_x = -1.5
        self.spawn_y = -0.5

        self.goal_reached = False

        # -----------------------------
        # LaserScan-data
        # -----------------------------
        self.scan_ranges = []
        self.angle_min = 0.0
        self.angle_increment = 0.0
        self.range_min = 0.0
        self.range_max = 3.5
        self.has_scan_received = False

        # -----------------------------
        # State machine
        # -----------------------------
        self.mode = "GO_TO_GOAL"
        self.avoid_direction = 0.0
        self.avoid_target_yaw = 0.0
        self.avoid_turn_start = None
        self.avoid_forward_start = None

        # -----------------------------
        # Publisher / Subscribers
        # -----------------------------
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
        if self.has_scan_received:
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

    def get_robot_position(self):
        if self.use_spawn_offset:
            robot_x = self.pose.position.x + self.spawn_x
            robot_y = self.pose.position.y + self.spawn_y
        else:
            robot_x = self.pose.position.x
            robot_y = self.pose.position.y

        return robot_x, robot_y

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

            # Framför roboten: ungefär -45 till +45 grader
            if -0.55 <= angle <= 0.55:
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
    def choose_avoid_direction(self, closest_angle, goal_angle):
        # Kolla vilken sida som är mest fri
       if closest_angle > 0:
           return -1.0
       else:
           return 1.0

    # --------------------------------------------------
    # Huvudlogik
    # --------------------------------------------------
    def control_robot(self):
        twist = Twist()

        if self.goal_reached:
            self.stop_robot()
            return

        # -----------------------------
        # Position och mål
        # -----------------------------
        robot_x, robot_y = self.get_robot_position()

        dx = self.goal_x - robot_x
        dy = self.goal_y - robot_y

        distance_to_goal = math.sqrt(dx**2 + dy**2)
        goal_angle = math.atan2(dy, dx)
        e_theta_goal = self.normalize_angle(goal_angle - self.yaw)

        # Lite större tolerans så roboten inte fastnar nära hinder vid målet
        if distance_to_goal < 0.35:
            self.goal_reached = True
            self.stop_robot()
            self.get_logger().info("Goal reached! Robot stopped.")
            return

        # -----------------------------
        # Hinder framför roboten
        # -----------------------------
        closest_distance, closest_angle = self.get_front_obstacle()

        self.get_logger().info(
            f"Mode: {self.mode}, Goal dist: {distance_to_goal:.2f}, "
            f"Obstacle: {closest_distance:.2f}, angle: {closest_angle:.2f}"
        )

        # ==================================================
        # MODE 1: GO_TO_GOAL
        # ==================================================
        if self.mode == "GO_TO_GOAL":
            if closest_distance < self.stop_distance:
                self.avoid_direction = self.choose_avoid_direction(
                    closest_angle,
                    goal_angle,
                )

                # Sväng ungefär 60 grader, inte 90.
                # Det gör att roboten inte vänder bort för mycket från målet.
                self.avoid_target_yaw = self.normalize_angle(
                    self.yaw + self.avoid_direction * math.pi / 3
                )

                self.mode = "AVOID_TURN"
                self.avoid_turn_start = self.get_clock().now()

                self.get_logger().info(
                    f"Obstacle detected. Switch to AVOID_TURN. "
                    f"Direction: {self.avoid_direction:.0f}"
                )

                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                return

            # P-regulator mot mål
            P = 0.8
            max_angular_speed = 0.30

            twist.angular.z = P * e_theta_goal
            twist.angular.z = max(
                min(twist.angular.z, max_angular_speed),
                -max_angular_speed,
            )

            if abs(e_theta_goal) > 0.60:
                twist.linear.x = 0.0
            elif abs(e_theta_goal) > 0.25:
                twist.linear.x = 0.05
            else:
                twist.linear.x = 0.10

            self.cmd_vel_pub.publish(twist)
            return

        # ==================================================
        # MODE 2: AVOID_TURN
        # ==================================================
        if self.mode == "AVOID_TURN":
            e_theta_avoid = self.normalize_angle(self.avoid_target_yaw - self.yaw)

            twist.linear.x = 0.0
            twist.angular.z = 0.8 * e_theta_avoid
            twist.angular.z = max(min(twist.angular.z, 0.35), -0.35)

            now = self.get_clock().now()
            elapsed_turn = (now - self.avoid_turn_start).nanoseconds / 1e9

            # Gå vidare om roboten nästan svängt klart eller om den svängt för länge
            if abs(e_theta_avoid) < 0.15 or elapsed_turn > 2.0:
                self.mode = "AVOID_FORWARD"
                self.avoid_forward_start = self.get_clock().now()
                twist.angular.z = 0.0

                self.get_logger().info("Finished turn. Switch to AVOID_FORWARD.")

            self.cmd_vel_pub.publish(twist)
            return

        # ==================================================
        # MODE 3: AVOID_FORWARD
        # ==================================================
        if self.mode == "AVOID_FORWARD":
            now = self.get_clock().now()
            elapsed = (now - self.avoid_forward_start).nanoseconds / 1e9

            # Kolla om det är fritt rakt framför
            front_clearance = self.get_sector_distance(-0.35, 0.35)

            if front_clearance < self.stop_distance + 0.15:
                self.mode = "AVOID_TURN"
                self.avoid_target_yaw = self.normalize_angle(
                    self.yaw + self.avoid_direction * math.pi / 4
                )
                self.avoid_turn_start = self.get_clock().now()

                twist.linear.x = 0.0
                twist.angular.z = 0.0

                self.get_logger().info(
                    f"Front still blocked: {front_clearance:.2f}. Turning more."
                )

                self.cmd_vel_pub.publish(twist)
                return

            # Kör försiktigt framåt förbi hindret
            twist.linear.x = 0.035

            # Styr lite tillbaka mot målet
            twist.angular.z = 0.15 * e_theta_goal
            twist.angular.z = max(min(twist.angular.z, 0.20), -0.20)

            # Om något fortfarande är rakt framför: sväng lite mer
            if closest_distance < self.stop_distance and abs(closest_angle) < 0.45:
                self.mode = "AVOID_TURN"
                self.avoid_target_yaw = self.normalize_angle(
                    self.yaw + self.avoid_direction * math.pi / 4
                )
                self.avoid_turn_start = self.get_clock().now()

                twist.linear.x = 0.0
                twist.angular.z = 0.0

                self.get_logger().info("Still blocked. Turning more.")
                self.cmd_vel_pub.publish(twist)
                return

            # Efter en kort stund: tillbaka till målet
            if elapsed > 2.0:
                self.mode = "GO_TO_GOAL"
                twist.linear.x = 0.0
                twist.angular.z = 0.0

                self.get_logger().info("Avoid forward done. Back to GO_TO_GOAL.")

            self.cmd_vel_pub.publish(twist)
            return

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