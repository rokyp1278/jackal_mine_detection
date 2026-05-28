#!/usr/bin/env python3
"""
automatic_driving_node.py
팀원(승민) 작성 — Frontier 탐사 + 장애물 회피 + AprilTag 접근/기록
버그 수정: topic명 /apriltag/detections, detection.id 접근 방식

구독:
  /map              (nav_msgs/OccupancyGrid)
  /scan             (sensor_msgs/LaserScan)
  /apriltag/detections (apriltag_msgs/AprilTagDetectionArray)

발행:
  /cmd_vel          (geometry_msgs/Twist)
"""

import math
import random

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan

from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion

try:
    from apriltag_msgs.msg import AprilTagDetectionArray
except ImportError:
    AprilTagDetectionArray = None


class FrontierAprilTagExplorer(Node):
    def __init__(self):
        super().__init__("automatic_driving_node")

        self.map_topic   = "/map"
        self.scan_topic  = "/scan"
        self.cmd_vel_topic = "/cmd_vel"
        self.tag_topic   = "/apriltag/detections"   # ← 수정: /tag_detections → /apriltag/detections

        self.robot_frame = "base_link"
        self.map_frame   = "map"

        self.max_linear_speed  = 0.25
        self.max_angular_speed = 0.6

        self.front_obstacle_distance = 0.7

        self.target_tag_distance    = 1.0
        self.tag_distance_tolerance = 0.15
        self.tag_center_tolerance   = 0.08

        self.kp_angular = 1.2
        self.kp_linear  = 0.4

        self.goal_tolerance = 0.4

        self.state = "SEARCH_FRONTIER"

        self.map_data      = None
        self.map_width     = None
        self.map_height    = None
        self.map_resolution = None
        self.map_origin_x  = None
        self.map_origin_y  = None

        self.current_goal = None

        self.front_min_range = float("inf")

        self.tag_detected      = False
        self.tag_center_error  = 0.0
        self.tag_distance      = None
        self.current_tag_id    = None
        self.visited_tags      = set()

        self.last_tag_seen_time = None

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.map_sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self.map_callback, 10)

        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, 10)

        if AprilTagDetectionArray is not None:
            self.tag_sub = self.create_subscription(
                AprilTagDetectionArray, self.tag_topic, self.tag_callback, 10)
        else:
            self.get_logger().warn("apriltag_msgs 없음 — AprilTag 기능 비활성화")

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Automatic Driving Node Started")

    # ── 콜백 ──────────────────────────────────────────────────────────
    def map_callback(self, msg: OccupancyGrid):
        self.map_data      = list(msg.data)
        self.map_width     = msg.info.width
        self.map_height    = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin_x  = msg.info.origin.position.x
        self.map_origin_y  = msg.info.origin.position.y

    def scan_callback(self, msg: LaserScan):
        ranges = list(msg.ranges)
        total  = len(ranges)
        center = total // 2
        window = total // 12

        front_ranges = ranges[center - window:center + window]
        front_valid  = [r for r in front_ranges if not math.isinf(r) and not math.isnan(r)]

        self.front_min_range = min(front_valid) if front_valid else float("inf")

    def tag_callback(self, msg):
        if len(msg.detections) == 0:
            self.tag_detected = False
            return

        detection = msg.detections[0]

        # ← 수정: detection.id[0] → detection.id (현재 apriltag_msgs 버전)
        try:
            tag_id = detection.id if isinstance(detection.id, int) else detection.id[0]
        except Exception:
            tag_id = 0

        if tag_id in self.visited_tags:
            self.tag_detected = False
            return

        self.current_tag_id     = tag_id
        self.tag_detected       = True
        self.last_tag_seen_time = self.get_clock().now()

        try:
            pose = detection.pose.pose.pose
            x = pose.position.x
            z = pose.position.z
            self.tag_center_error = x / max(z, 0.001)
            self.tag_distance     = z
        except Exception:
            self.get_logger().warn("AprilTag pose 읽기 실패")
            self.tag_center_error = 0.0
            self.tag_distance     = None

    # ── 메인 루프 ─────────────────────────────────────────────────────
    def control_loop(self):
        if self.map_data is None:
            self.stop_robot()
            return

        if self.tag_detected and self.state not in ["ALIGN_TAG", "APPROACH_TAG", "CAPTURE_TAG"]:
            self.get_logger().info(f"AprilTag 감지: ID {self.current_tag_id}")
            self.state = "ALIGN_TAG"

        if   self.state == "SEARCH_FRONTIER":   self.search_frontier_state()
        elif self.state == "MOVE_TO_FRONTIER":  self.move_to_frontier_state()
        elif self.state == "ALIGN_TAG":         self.align_tag_state()
        elif self.state == "APPROACH_TAG":      self.approach_tag_state()
        elif self.state == "CAPTURE_TAG":       self.capture_tag_state()
        elif self.state == "AVOID_OBSTACLE":    self.avoid_obstacle_state()
        else:
            self.stop_robot()
            self.state = "SEARCH_FRONTIER"

    # ── 상태 함수 ─────────────────────────────────────────────────────
    def search_frontier_state(self):
        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            self.stop_robot()
            return

        frontiers = self.find_frontiers()
        if not frontiers:
            self.get_logger().info("Frontier 없음 — 탐사 완료")
            self.stop_robot()
            return

        rx, ry, _ = robot_pose
        self.current_goal = min(frontiers, key=lambda p: math.hypot(p[0]-rx, p[1]-ry))
        self.get_logger().info(
            f"새 Frontier: ({self.current_goal[0]:.2f}, {self.current_goal[1]:.2f})")
        self.state = "MOVE_TO_FRONTIER"

    def move_to_frontier_state(self):
        if self.current_goal is None:
            self.state = "SEARCH_FRONTIER"
            return

        if self.front_min_range < self.front_obstacle_distance:
            self.state = "AVOID_OBSTACLE"
            return

        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            self.stop_robot()
            return

        rx, ry, yaw = robot_pose
        gx, gy = self.current_goal
        distance  = math.hypot(gx-rx, gy-ry)
        yaw_error = self.normalize_angle(math.atan2(gy-ry, gx-rx) - yaw)

        if distance < self.goal_tolerance:
            self.get_logger().info("Frontier 도달")
            self.current_goal = None
            self.state = "SEARCH_FRONTIER"
            return

        cmd = Twist()
        if abs(yaw_error) > 0.4:
            cmd.angular.z = self.clamp(1.2*yaw_error, -self.max_angular_speed, self.max_angular_speed)
        else:
            cmd.linear.x  = self.max_linear_speed
            cmd.angular.z = self.clamp(0.8*yaw_error, -self.max_angular_speed, self.max_angular_speed)
        self.cmd_pub.publish(cmd)

    def align_tag_state(self):
        if not self.tag_detected:
            self.state = "SEARCH_FRONTIER"
            return

        if abs(self.tag_center_error) < self.tag_center_tolerance:
            self.stop_robot()
            self.state = "APPROACH_TAG"
            return

        cmd = Twist()
        cmd.angular.z = self.clamp(-self.kp_angular*self.tag_center_error,
                                   -self.max_angular_speed, self.max_angular_speed)
        self.cmd_pub.publish(cmd)

    def approach_tag_state(self):
        if not self.tag_detected or self.tag_distance is None:
            self.state = "SEARCH_FRONTIER"
            return

        if self.front_min_range < 0.4:
            self.stop_robot()
            self.state = "CAPTURE_TAG"
            return

        dist_err   = self.tag_distance - self.target_tag_distance
        center_err = self.tag_center_error

        if abs(dist_err) < self.tag_distance_tolerance and abs(center_err) < self.tag_center_tolerance:
            self.stop_robot()
            self.state = "CAPTURE_TAG"
            return

        cmd = Twist()
        cmd.linear.x  = self.clamp(self.kp_linear*dist_err, -0.15, 0.20)
        cmd.angular.z = self.clamp(-self.kp_angular*center_err, -0.4, 0.4)
        self.cmd_pub.publish(cmd)

    def capture_tag_state(self):
        self.stop_robot()
        if self.current_tag_id is not None:
            self.visited_tags.add(self.current_tag_id)
            self.get_logger().info(f"Tag {self.current_tag_id} 기록 완료")
        self.tag_detected     = False
        self.current_tag_id   = None
        self.tag_distance     = None
        self.tag_center_error = 0.0
        self.state = "SEARCH_FRONTIER"

    def avoid_obstacle_state(self):
        cmd = Twist()
        if self.front_min_range < self.front_obstacle_distance:
            cmd.angular.z = 0.45
            self.cmd_pub.publish(cmd)
        else:
            self.stop_robot()
            self.state = "MOVE_TO_FRONTIER"

    # ── 유틸 ──────────────────────────────────────────────────────────
    def find_frontiers(self):
        frontiers = []
        for y in range(1, self.map_height-1):
            for x in range(1, self.map_width-1):
                if self.map_data[self.grid_to_index(x,y)] != 0:
                    continue
                if self.is_next_to_unknown(x, y):
                    frontiers.append(self.grid_to_world(x, y))
        if len(frontiers) > 300:
            frontiers = random.sample(frontiers, 300)
        return frontiers

    def is_next_to_unknown(self, x, y):
        for nx, ny in [(x+1,y),(x-1,y),(x,y+1),(x,y-1),(x+1,y+1),(x-1,y-1),(x+1,y-1),(x-1,y+1)]:
            if 0 <= nx < self.map_width and 0 <= ny < self.map_height:
                if self.map_data[self.grid_to_index(nx, ny)] == -1:
                    return True
        return False

    def get_robot_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, rclpy.time.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            q = trans.transform.rotation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            return x, y, yaw
        except Exception:
            return None

    def grid_to_index(self, x, y): return y * self.map_width + x
    def grid_to_world(self, gx, gy):
        return (self.map_origin_x + gx*self.map_resolution,
                self.map_origin_y + gy*self.map_resolution)

    def normalize_angle(self, a):
        while a >  math.pi: a -= 2*math.pi
        while a < -math.pi: a += 2*math.pi
        return a

    def clamp(self, v, lo, hi): return max(min(v, hi), lo)

    def stop_robot(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = FrontierAprilTagExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.stop_robot()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
