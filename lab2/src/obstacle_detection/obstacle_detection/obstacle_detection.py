#! /usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile

from geometry_msgs.msg import TwistStamped, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion


class ObstacleDetection(Node):
    def __init__(self):
        super().__init__("obstacle_detection")
        self.get_logger().info("JAG KÖR FÄRDIG VERSION: MÅL + HINDER + KORRIDOR + ESCAPE")

        # -----------------------------
        # Parametrar
        # -----------------------------
        self.declare_parameter("stop_distance", 0.20)
        self.declare_parameter("goal_x", 3.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("goal_tolerance", 0.35)

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

        self.goal_tolerance = (
            self.get_parameter("goal_tolerance")
            .get_parameter_value()
            .double_value
        )

        self.get_logger().info(f"Using stop_distance: {self.stop_distance}")
        self.get_logger().info(f"Using goal_tolerance: {self.goal_tolerance}")
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
        self.last_turn_direction = 1.0

        qos = QoSProfile(depth=10)

        self.cmd_vel_pub = self.create_publisher(TwistStamped, "cmd_vel", qos)

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
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        msg.twist.linear.x = 0.0
        msg.twist.angular.z = 0.0

        self.cmd_vel_pub.publish(msg)

    def valid_range(self, r):
        if math.isinf(r) or math.isnan(r):
            return False

        if r < self.range_min or r > self.range_max:
            return False

        return True

    # --------------------------------------------------
    # Hitta närmaste hinder i en sektor
    # Vinklarna är i radianer.
    # Framåt är ungefär 0 rad.
    # Vänster är positiv vinkel.
    # Höger är negativ vinkel.
    # --------------------------------------------------
    def get_sector_distance(self, min_angle, max_angle):
        closest_distance = float("inf")

        for i, r in enumerate(self.scan_ranges):
            if not self.valid_range(r):
                continue

            angle = self.angle_min + i * self.angle_increment
            angle = self.normalize_angle(angle)

            if min_angle <= angle <= max_angle:
                if r < closest_distance:
                    closest_distance = r

        return closest_distance

    # --------------------------------------------------
    # Skicka hastighet
    # --------------------------------------------------
    def publish_cmd(self, msg, mode, extra_info=""):
        self.cmd_vel_pub.publish(msg)

        self.get_logger().info(
            f"{mode} | v={msg.twist.linear.x:.2f}, w={msg.twist.angular.z:.2f} {extra_info}"
        )

    # --------------------------------------------------
    # Huvudlogik
    # --------------------------------------------------
    def control_robot(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        if self.goal_reached:
            self.stop_robot()
            return

        # -----------------------------
        # Position och mål
        # -----------------------------
        robot_x = self.pose.position.x
        robot_y = self.pose.position.y

        dx = self.goal_x - robot_x
        dy = self.goal_y - robot_y

        distance_to_goal = math.sqrt(dx**2 + dy**2)
        goal_angle = math.atan2(dy, dx)
        e_theta_goal = self.normalize_angle(goal_angle - self.yaw)

        # -----------------------------
        # Mål nått
        # -----------------------------
        if distance_to_goal < self.goal_tolerance:
            self.goal_reached = True
            self.stop_robot()
            self.get_logger().info("Goal reached! Robot stopped.")
            return

        # -----------------------------
        # LIDAR-sektorer
        # -----------------------------
        front = self.get_sector_distance(-0.45, 0.45)
        front_left = self.get_sector_distance(0.15, 0.85)
        front_right = self.get_sector_distance(-0.85, -0.15)
        left = self.get_sector_distance(0.65, 1.40)
        right = self.get_sector_distance(-1.40, -0.65)

        # -----------------------------
        # Avståndsgränser
        # -----------------------------
        danger_distance = self.stop_distance       # akut nära hinder
        front_distance = 0.32                      # börja väja framför
        side_distance = 0.23                       # nära vägg
        corner_distance = 0.23                     # hörn/fast-läge

        # --------------------------------------------------
        # 0. ESCAPE-LÄGE
        # Om roboten står i hörn eller nära vägg + hinder framför:
        # backa lite och rotera mot friaste sidan.
        # --------------------------------------------------
        if front < corner_distance and (left < corner_distance or right < corner_distance):
            msg.twist.linear.x = -0.035

            if left > right:
                msg.twist.angular.z = 0.45
                self.last_turn_direction = 1.0
            elif right > left:
                msg.twist.angular.z = -0.45
                self.last_turn_direction = -1.0
            else:
                msg.twist.angular.z = 0.45 * self.last_turn_direction

            self.publish_cmd(
                msg,
                "CORNER ESCAPE",
                f"| front={front:.2f}, left={left:.2f}, right={right:.2f}"
            )
            return

        # --------------------------------------------------
        # 1. Akut hinder rakt framför
        # Stoppa framåt och rotera mot sidan där det finns mest plats.
        # --------------------------------------------------
        if front < danger_distance:
            msg.twist.linear.x = 0.0

            if left > right:
                msg.twist.angular.z = 0.45
                self.last_turn_direction = 1.0
            elif right > left:
                msg.twist.angular.z = -0.45
                self.last_turn_direction = -1.0
            else:
                msg.twist.angular.z = 0.45 * self.last_turn_direction

            self.publish_cmd(
                msg,
                "DANGER FRONT",
                f"| front={front:.2f}, left={left:.2f}, right={right:.2f}"
            )
            return

        # --------------------------------------------------
        # 2. Hinder framför men inte akut
        # Kör långsamt och sväng mot friaste fram-sidan.
        # --------------------------------------------------
        if front < front_distance:
            msg.twist.linear.x = 0.025

            if front_left > front_right:
                msg.twist.angular.z = 0.35
                self.last_turn_direction = 1.0
            elif front_right > front_left:
                msg.twist.angular.z = -0.35
                self.last_turn_direction = -1.0
            else:
                msg.twist.angular.z = 0.35 * self.last_turn_direction

            self.publish_cmd(
                msg,
                "AVOID FRONT",
                f"| front={front:.2f}, front_left={front_left:.2f}, front_right={front_right:.2f}"
            )
            return

        # --------------------------------------------------
        # 3. För nära vänster vägg
        # Fortsätt sakta framåt men styr höger.
        # --------------------------------------------------
        if left < side_distance:
            msg.twist.linear.x = 0.04
            msg.twist.angular.z = -0.28

            self.publish_cmd(
                msg,
                "TOO CLOSE LEFT",
                f"| left={left:.2f}, right={right:.2f}"
            )
            return

        # --------------------------------------------------
        # 4. För nära höger vägg
        # Fortsätt sakta framåt men styr vänster.
        # --------------------------------------------------
        if right < side_distance:
            msg.twist.linear.x = 0.04
            msg.twist.angular.z = 0.28

            self.publish_cmd(
                msg,
                "TOO CLOSE RIGHT",
                f"| left={left:.2f}, right={right:.2f}"
            )
            return

        # --------------------------------------------------
        # 5. Korridorkörning
        # Om väggar finns på båda sidor: håll ungefär mitten,
        # men glöm inte målet.
        # --------------------------------------------------
        in_corridor = left < 1.0 and right < 1.0

        if in_corridor:
            # Om right - left är positivt:
            # höger sida har mer plats än vänster, roboten är närmare vänster vägg.
            # Då behöver den svänga höger, alltså negativ angular.z.
            wall_error = left - right

            wall_correction = 0.30 * wall_error
            wall_correction = max(min(wall_correction, 0.22), -0.22)

            P_goal = 0.65
            angular = P_goal * e_theta_goal + wall_correction
            angular = max(min(angular, 0.35), -0.35)

            msg.twist.angular.z = angular

            if abs(e_theta_goal) > 0.80:
                msg.twist.linear.x = 0.025
            elif abs(e_theta_goal) > 0.45:
                msg.twist.linear.x = 0.045
            else:
                msg.twist.linear.x = 0.065

            self.publish_cmd(
                msg,
                "CORRIDOR",
                f"| left={left:.2f}, right={right:.2f}, e_goal={e_theta_goal:.2f}, dist={distance_to_goal:.2f}"
            )
            return

        # --------------------------------------------------
        # 6. Fri väg: kör mot målet
        # --------------------------------------------------
        P = 1.0
        angular = P * e_theta_goal
        angular = max(min(angular, 0.40), -0.40)

        msg.twist.angular.z = angular

        if abs(e_theta_goal) > 0.85:
            msg.twist.linear.x = 0.0
        elif abs(e_theta_goal) > 0.45:
            msg.twist.linear.x = 0.04
        else:
            msg.twist.linear.x = 0.085

        self.publish_cmd(
            msg,
            "GOAL",
            f"| robot=({robot_x:.2f},{robot_y:.2f}), "
            f"goal=({self.goal_x:.2f},{self.goal_y:.2f}), "
            f"dist={distance_to_goal:.2f}, e_goal={e_theta_goal:.2f}, "
            f"front={front:.2f}, left={left:.2f}, right={right:.2f}"
        )

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