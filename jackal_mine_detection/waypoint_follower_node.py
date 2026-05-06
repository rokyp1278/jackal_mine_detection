#!/usr/bin/env python3
"""
waypoint_follower_node.py  (3단계 완전 구현)

params 의 waypoints 리스트를 Nav2 NavigateToPose 로 순서대로 방문한 후,
완료 시 /finish_exploration std_msgs/Bool(True) 를 발행하여
exploration_manager_node 에 탐사 종료를 알린다.

파라미터:
  waypoints:           double[]   [x1,y1,yaw1_rad, x2,y2,yaw2_rad, ...]
  nav2_action:         str        "navigate_to_pose"
  waypoint_timeout_sec:float      60.0
  map_frame:           str        "map"
  auto_start:          bool       true   (시작 즉시 탐사 시작)
  start_delay_sec:     float      5.0    (Nav2 기동 대기 시간)
"""
import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

from jackal_mine_detection.utils.geometry import quat_from_yaw


class WaypointFollowerNode(Node):
    def __init__(self):
        super().__init__('waypoint_follower_node')

        # ── 파라미터 ──────────────────────────────────────────────────
        self.declare_parameter('waypoints', [
            # 기본값: 간단한 L자형 경로
            1.0,  0.0,  0.0,
            3.0,  0.5,  1.5708,
            6.0,  0.0,  0.0,
            8.0, -0.5, -1.5708,
        ])
        self.declare_parameter('nav2_action', 'navigate_to_pose')
        self.declare_parameter('waypoint_timeout_sec', 60.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('auto_start', True)
        self.declare_parameter('start_delay_sec', 5.0)

        flat              = list(self.get_parameter('waypoints').value)
        action_name       = str(self.get_parameter('nav2_action').value)
        self.wp_timeout   = float(self.get_parameter('waypoint_timeout_sec').value)
        self.map_frame    = str(self.get_parameter('map_frame').value)
        self.auto_start   = bool(self.get_parameter('auto_start').value)
        start_delay       = float(self.get_parameter('start_delay_sec').value)

        # 3개씩 끊기
        if len(flat) % 3 != 0:
            self.get_logger().warn(
                f'waypoints 길이({len(flat)})가 3의 배수가 아님, 잘림'
            )
            flat = flat[: (len(flat) // 3) * 3]

        self.waypoints: list[tuple[float, float, float]] = [
            (float(flat[i]), float(flat[i + 1]), float(flat[i + 2]))
            for i in range(0, len(flat), 3)
        ]
        self._idx       = 0
        self._active    = False
        self._started   = False

        # ── Publisher ─────────────────────────────────────────────────
        self._pub_finish = self.create_publisher(Bool,   '/finish_exploration',      10)
        self._pub_marker = self.create_publisher(Marker, '/current_waypoint_marker', 10)

        # ── Action client ─────────────────────────────────────────────
        self._ac = ActionClient(self, NavigateToPose, action_name)

        self.get_logger().info(
            f'waypoint_follower_node 시작. {len(self.waypoints)}개 waypoint, '
            f'auto_start={self.auto_start}, start_delay={start_delay}s'
        )

        if self.auto_start:
            # Nav2 기동을 기다린 뒤 시작 (one-shot 타이머)
            self._start_timer = self.create_timer(start_delay, self._delayed_start)

    # ─────────────────────────────────────────────────────────────────
    def _delayed_start(self):
        """start_delay_sec 후 한 번만 호출되는 one-shot 콜백."""
        if self._started:
            return
        self._started = True
        self._start_timer.cancel()
        self.get_logger().info('Waypoint exploration 시작...')
        self._send_next()

    # ─────────────────────────────────────────────────────────────────
    def _send_next(self):
        if self._idx >= len(self.waypoints):
            self.get_logger().info(
                f'모든 waypoint 완료! /finish_exploration=True 발행'
            )
            msg = Bool()
            msg.data = True
            self._pub_finish.publish(msg)
            return

        x, y, yaw = self.waypoints[self._idx]
        self.get_logger().info(
            f'[WP {self._idx + 1}/{len(self.waypoints)}] '
            f'→ ({x:.2f}, {y:.2f}, yaw={math.degrees(yaw):.1f}°)'
        )
        self._publish_marker(x, y, yaw)

        # Nav2 action server 대기
        if not self._ac.wait_for_server(timeout_sec=15.0):
            self.get_logger().error(
                f'Nav2 action server 타임아웃! WP {self._idx + 1} 스킵'
            )
            self._idx += 1
            self._active = False
            self._send_next()
            return

        # Goal 구성
        goal = NavigateToPose.Goal()
        goal.pose.header.stamp    = self.get_clock().now().to_msg()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0
        qx, qy, qz, qw = quat_from_yaw(yaw)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self._active = True
        future = self._ac.send_goal_async(
            goal, feedback_callback=self._on_feedback
        )
        future.add_done_callback(self._on_goal_response)

    # ─────────────────────────────────────────────────────────────────
    def _on_feedback(self, fb_msg):
        d = fb_msg.feedback.distance_remaining
        self.get_logger().info(
            f'[WP {self._idx + 1}] 남은 거리: {d:.2f}m',
            throttle_duration_sec=5.0,
        )

    def _on_goal_response(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().warn(
                f'WP {self._idx + 1} Nav2 rejected, 스킵'
            )
            self._idx += 1
            self._active = False
            self._send_next()
            return
        result_future = gh.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future):
        result = future.result()
        status = result.status
        label  = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}.get(
            status, f'UNKNOWN({status})'
        )
        self.get_logger().info(f'WP {self._idx + 1} 완료: {label}')
        self._idx  += 1
        self._active = False
        self._send_next()

    # ─────────────────────────────────────────────────────────────────
    def _publish_marker(self, x: float, y: float, yaw: float):
        m = Marker()
        m.header.stamp    = self.get_clock().now().to_msg()
        m.header.frame_id = self.map_frame
        m.ns    = 'current_waypoint'
        m.id    = 0
        m.type  = Marker.ARROW
        m.action= Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.3
        qx, qy, qz, qw = quat_from_yaw(yaw)
        m.pose.orientation.x = qx
        m.pose.orientation.y = qy
        m.pose.orientation.z = qz
        m.pose.orientation.w = qw
        m.scale.x = 0.6
        m.scale.y = 0.12
        m.scale.z = 0.12
        m.color   = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.9)
        self._pub_marker.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
