from rclpy.qos import qos_profile_sensor_data
import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import TwistStamped, PoseWithCovarianceStamped
from sensor_msgs import msg
from sensor_msgs.msg import LaserScan


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class PathFollower(Node):
    def __init__(self):
        super().__init__("path_follower")
        self.get_logger().info(
            "Path follower started: strict A* path + safe corridor navigation"
        )

        self.cmd_pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)

        # Path state
        self.path = []
        self.target_index = 1
        self.goal_logged = False

        # Robot state from AMCL/map
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.has_pose = False

        # Laser state
        self.scan_ranges = []
        self.angle_min = 0.0
        self.angle_increment = 0.0
        self.range_min = 0.0
        self.range_max = 3.5
        self.has_scan = False

        # Distances
        self.front_distance = float("inf")
        self.left_distance = float("inf")
        self.right_distance = float("inf")

        # Avoidance memory
        self.last_turn_direction = 1.0

        self.path_sub = self.create_subscription(
            Path,
            "/planned_path",
            self.path_callback,
            10
        )

        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.amcl_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            qos_profile_sensor_data
        )

        self.timer = self.create_timer(0.1, self.control_loop)

    def path_callback(self, msg):
    # Ignore empty paths
        if not msg.poses:
            return

    # If this is the same path as before, do not reset target_index.
    # astar_planner republishes the same path, and resetting here
    # makes the robot start over again and shake.
        if self.path:
            old_first = self.path[0].pose.position
            old_last = self.path[-1].pose.position
            new_first = msg.poses[0].pose.position
            new_last = msg.poses[-1].pose.position

            same_length = len(self.path) == len(msg.poses)
            same_start = abs(old_first.x - new_first.x) < 0.001 and abs(old_first.y - new_first.y) < 0.001
            same_goal = abs(old_last.x - new_last.x) < 0.001 and abs(old_last.y - new_last.y) < 0.001

            if same_length and same_start and same_goal:
                return

        self.path = msg.poses
        self.target_index = 0
        self.goal_logged = False

        self.get_logger().info(f"Saved NEW path with {len(self.path)} poses")

        first_pose = self.path[0].pose.position
        last_pose = self.path[-1].pose.position

        self.get_logger().info(
        f"First point: x={first_pose.x:.3f}, y={first_pose.y:.3f}"
        )
        self.get_logger().info(
            f"Last point: x={last_pose.x:.3f}, y={last_pose.y:.3f}"
        )

    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment
        self.range_min = msg.range_min
        self.range_max = msg.range_max
        self.has_scan = True

    def amcl_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.has_pose = True

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def valid_range(self, r):
        if math.isinf(r) or math.isnan(r):
            return False

        if r < self.range_min or r > self.range_max:
            return False

        return True

    def get_sector_distance(self, min_angle, max_angle):
        closest_distance = float("inf")

        for i, r in enumerate(self.scan_ranges):
            if not self.valid_range(r):
                continue

            angle = self.angle_min + i * self.angle_increment
            angle = self.normalize_angle(angle)

            if min_angle <= angle <= max_angle:
                closest_distance = min(closest_distance, r)

        return closest_distance

    def make_twist(self, linear_x=0.0, angular_z=0.0):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        msg.twist.linear.x = linear_x
        msg.twist.angular.z = angular_z

        return msg

    def publish_stop(self):
        self.cmd_pub.publish(self.make_twist(0.0, 0.0))

    def publish_cmd(self, msg, mode, extra_info=""):
        self.cmd_pub.publish(msg)

        self.get_logger().info(
            f"{mode} | v={msg.twist.linear.x:.3f}, "
            f"w={msg.twist.angular.z:.3f} {extra_info}"
        )

    def update_target_index(self):
        if not self.path:
            return

    # Följ pathen strikt, punkt för punkt.
    # Hoppa inte långt fram i pathen, eftersom det gör att roboten siktar mot mål
    # eller mot en punkt på andra sidan väggen.
        while self.target_index < len(self.path) - 1:
            p = self.path[self.target_index].pose.position

            d = math.sqrt(
                (p.x - self.robot_x) ** 2 +
                (p.y - self.robot_y) ** 2
            )

            if d < 0.12:
                self.target_index += 1
            else:
                break

    def control_loop(self):
        if not self.path:
            return

        if not self.has_pose or not self.has_scan:
            return

        self.update_target_index()

        target_pose = self.path[self.target_index].pose.position

        dx = target_pose.x - self.robot_x
        dy = target_pose.y - self.robot_y

        distance = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)
        angle_error = self.normalize_angle(target_angle - self.robot_yaw)

        # Final goal reached
        if self.target_index == len(self.path) - 1 and distance < 0.20:
            self.publish_stop()

            if not self.goal_logged:
                self.get_logger().info("Final goal reached safely. Stopping robot.")
                self.goal_logged = True

            return

        # Move to next target point
        if distance < 0.30 and self.target_index < len(self.path) - 1:
            self.target_index += 1
            self.get_logger().info(
                f"Moving to next target index: {self.target_index}"
            )
            return

        # LIDAR sectors
        front = self.get_sector_distance(-0.45, 0.45)
        front_left = self.get_sector_distance(0.15, 0.85)
        front_right = self.get_sector_distance(-0.85, -0.15)
        left = self.get_sector_distance(0.65, 1.40)
        right = self.get_sector_distance(-1.40, -0.65)

        self.front_distance = front
        self.left_distance = left
        self.right_distance = right

        # Tuned for narrow maze
        danger_distance = 0.13
        front_distance = 0.25
        side_distance = 0.20
        corner_distance = 0.15

        # 0. If path requires a big turn and wall is close ahead:
        # Rotate toward path first. Do not drive into the corner.
        if front < 0.22 and abs(angle_error) > 1.10:
            angular = 0.22 if angle_error > 0 else -0.22
            msg = self.make_twist(0.0, angular)

            self.publish_cmd(
                msg,
                "TURN TO PATH BEFORE MOVING",
                f"| index={self.target_index}, front={front:.2f}, "
                f"e_path={angle_error:.2f}, left={left:.2f}, right={right:.2f}"
            )
            return

        # 1. Corner / stuck escape
        if front < corner_distance and (left < corner_distance or right < corner_distance):
            if left > right:
                angular = 0.35
                self.last_turn_direction = 1.0
            elif right > left:
                angular = -0.35
                self.last_turn_direction = -1.0
            else:
                angular = 0.35 * self.last_turn_direction

            # Back very slowly only in real corner/stuck situation
            msg = self.make_twist(-0.015, angular)

            self.publish_cmd(
                msg,
                "CORNER ESCAPE",
                f"| front={front:.2f}, left={left:.2f}, right={right:.2f}"
            )
            return

        # 2. Danger directly in front
        if front < danger_distance:
            # Still use path direction if possible
            if abs(angle_error) > 0.25:
                angular = 0.35 if angle_error > 0 else -0.35
            elif left > right:
                angular = 0.35
                self.last_turn_direction = 1.0
            elif right > left:
                angular = -0.35
                self.last_turn_direction = -1.0
            else:
                angular = 0.35 * self.last_turn_direction

            msg = self.make_twist(0.0, angular)

            self.publish_cmd(
                msg,
                "DANGER FRONT + PATH",
                f"| front={front:.2f}, left={left:.2f}, right={right:.2f}, "
                f"e_path={angle_error:.2f}"
            )
            return

        # 3. Obstacle ahead, but not emergency:
        # Blend obstacle avoidance with path direction.
        if front < front_distance:
            angular = 0.9 * angle_error
            angular = max(min(angular, 0.20), -0.0)

            if abs(angle_error) > 0.35:
                linear = 0.0
            else:
                linear = 0.008

            msg = self.make_twist(linear, angular)

            self.publish_cmd(
                msg,
                "SLOW PATH FOLLOW NEAR FRONT",
                f"| front={front:.2f}, e_path={angle_error:.2f}"
            )
            return

        # 4. Too close to left wall.
        # But if path strongly says left, do not fight it too hard.
        if left < side_distance:
            wall_angular = -0.22
            path_angular = 0.55 * angle_error
            angular = 0.65 * wall_angular + 0.35 * path_angular
            angular = max(min(angular, 0.20), -0.20)

            msg = self.make_twist(0.020, angular)

            self.publish_cmd(
                msg,
                "TOO CLOSE LEFT + PATH",
                f"| left={left:.2f}, right={right:.2f}, e_path={angle_error:.2f}"
            )
            return

        # 5. Too close to right wall.
        # But if path strongly says right, do not fight it too hard.
        if right < side_distance:
            wall_angular = 0.22
            path_angular = 0.55 * angle_error
            angular = 0.65 * wall_angular + 0.35 * path_angular
            angular = max(min(angular, 0.30), -0.30)

            msg = self.make_twist(0.020, angular)

            self.publish_cmd(
                msg,
                "TOO CLOSE RIGHT + PATH",
                f"| left={left:.2f}, right={right:.2f}, e_path={angle_error:.2f}"
            )
            return
        # STRICT PATH FOLLOWING:
        # Om det inte finns akut hinder framför roboten, följ nästa waypoint strikt.
        # STRICT PATH FOLLOWING:
# Följ nästa waypoint långsamt men fortsätt röra dig om det finns plats framåt.
        # STRICT PATH FOLLOWING:
# I trånga svängar ska roboten först rotera mot pathen,
        # inte köra fram samtidigt och skära hörnet.
        # STRICT PATH FOLLOWING:
# Följ pathen långsamt. Vid sväng ska roboten inte stå helt stilla,
# eftersom den då kan fastna i hörnet. Den ska krypa framåt lite.
        if front > 0.12:
            angular = 1.15 * angle_error
            angular = max(min(angular, 0.32), -0.32)

            if abs(angle_error) > 1.20:
                # Mycket stor felvinkel: rotera nästan på plats
                linear = 0.00
            elif abs(angle_error) > 0.75:
                # Stor sväng: kryp mycket långsamt medan den svänger
                linear = 0.006
            elif abs(angle_error) > 0.50:
                # Medium sväng
                linear = 0.015
            elif abs(angle_error) > 0.25:
                # Liten sväng
                linear = 0.030
            else:
                # Bra riktning
                linear = 0.055

            # Om väggen är nära på sidan, kör extra långsamt
            if left < 0.18 or right < 0.16:
                linear = min(linear, 0.016)

            msg = self.make_twist(linear, angular)

            self.publish_cmd(
                msg,
                "STRICT PATH CREEP",
                f"| index={self.target_index}, dist={distance:.2f}, "
                f"e_path={angle_error:.2f}, front={front:.2f}, "
                f"left={left:.2f}, right={right:.2f}"
            )
            return

        # 6. Corridor mode
        in_corridor = left < 1.0 and right < 1.0

        if in_corridor:
            wall_error = left - right

            # Wall correction keeps robot away from walls,
            # but A* path is still the main guide.
            wall_correction = 0.16 * wall_error
            wall_correction = max(min(wall_correction, 0.10), -0.10)

            # If robot is far from path direction, ignore wall correction.
            if abs(angle_error) > 0.75:
                wall_correction = 0.0

            p_goal = 0.95
            angular = p_goal * angle_error + wall_correction
            angular = max(min(angular, 0.32), -0.32)

            # Important: during large turns, rotate first.
            if abs(angle_error) > 0.90:
                linear = 0.0
            elif abs(angle_error) > 0.60:
                linear = 0.012
            elif abs(angle_error) > 0.35:
                linear = 0.025
            else:
                linear = 0.045

            # Extra safety if close to walls
            if left < 0.28 or right < 0.28:
                linear = min(linear, 0.020)

            msg = self.make_twist(linear, angular)

            self.publish_cmd(
                msg,
                "CORRIDOR",
                f"| index={self.target_index}, dist={distance:.2f}, "
                f"e_path={angle_error:.2f}, left={left:.2f}, right={right:.2f}, "
                f"wall_corr={wall_correction:.2f}, front={front:.2f}"
            )
            return

        # 7. Free path following
        angular = 0.90 * angle_error
        angular = max(min(angular, 0.35), -0.35)

        if abs(angle_error) > 0.90:
            linear = 0.0
        elif abs(angle_error) > 0.55:
            linear = 0.020
        elif abs(angle_error) > 0.35:
            linear = 0.035
        else:
            linear = 0.060

        msg = self.make_twist(linear, angular)

        self.publish_cmd(
            msg,
            "PATH",
            f"| index={self.target_index}, dist={distance:.2f}, "
            f"e_path={angle_error:.2f}, front={front:.2f}, "
            f"left={left:.2f}, right={right:.2f}"
        )

    def destroy_node(self):
        self.get_logger().info("Shutting down, stopping robot...")

        try:
            if rclpy.ok():
                self.publish_stop()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = PathFollower()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt caught.")
        try:
            node.publish_stop()
        except Exception:
            pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()