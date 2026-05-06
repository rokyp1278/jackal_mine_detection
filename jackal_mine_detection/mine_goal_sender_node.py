"""
mine_goal_sender_node.py

Send a Nav2 NavigateToPose goal to /mine_cluster_center after exploration ends.
- Subscribes /trigger_final_goal (std_msgs/Empty) to start sending
- Skips resending if goal is closer than goal_resend_threshold to last goal
- Waits for Nav2 action server (/navigate_to_pose) before sending
- Logs SUCCEEDED / ABORTED / CANCELED on result

Params:
  nav2_action:           "/navigate_to_pose"
  goal_resend_threshold: 0.5  [m]
  approach_offset:       0.5  [m]
  goal_yaw_mode:         "face_center" or "keep_zero"
"""
from __future__ import annotations
import math
from typing import Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

from jackal_mine_detection.utils.geometry import quat_from_yaw


class MineGoalSenderNode(Node):
    def __init__(self):
        super().__init__('mine_goal_sender_node')

        self.declare_parameter('nav2_action', 'navigate_to_pose')
        self.declare_parameter('goal_resend_threshold', 0.5)
        self.declare_parameter('approach_offset', 0.5)
        self.declare_parameter('goal_yaw_mode', 'face_center')
        self.declare_parameter('cluster_topic', '/mine_cluster_center')
        self.declare_parameter('trigger_topic', '/trigger_final_goal')
        self.declare_parameter('robot_frame', 'base_link')

        action_name = str(self.get_parameter('nav2_action').value)
        self.resend_th = float(self.get_parameter('goal_resend_threshold').value)
        self.offset = float(self.get_parameter('approach_offset').value)
        self.yaw_mode = str(self.get_parameter('goal_yaw_mode').value)
        cluster_topic = str(self.get_parameter('cluster_topic').value)
        trigger_topic = str(self.get_parameter('trigger_topic').value)

        self._latest_center = None  # type: Optional[PoseStamped]
        self._last_goal_xy = None   # type: Optional[Tuple[float, float]]
        self._goal_in_flight = False

        self.create_subscription(
            PoseStamped, cluster_topic, self._cb_center, 10
        )
        self.create_subscription(Empty, trigger_topic, self._cb_trigger, 10)

        self._action_client = ActionClient(self, NavigateToPose, action_name)
        self.get_logger().info(
            'mine_goal_sender_node up. waiting Nav2 action "%s" ...' % action_name
        )

    def _cb_center(self, msg):
        self._latest_center = msg

    def _cb_trigger(self, _msg):
        if self._latest_center is None:
            self.get_logger().warn(
                'trigger received but no /mine_cluster_center yet'
            )
            return
        if self._goal_in_flight:
            self.get_logger().info('goal already in flight, ignore trigger')
            return
        self._send_goal_for_center(self._latest_center)

    def _build_goal_pose(self, center):
        cx = center.pose.position.x
        cy = center.pose.position.y

        # 동쪽에서 접근: WP4(x≈10) 완료 후 로봇이 동쪽에 있으므로
        # 클러스터 동쪽 offset 위치(복도 중앙 y=0)에서 서쪽(클러스터 방향)을 향해 정지.
        # x=3 부근 SLAM 노이즈 셀을 회피하고 짧은 경로(9.9→4.5)만 주행.
        gx = cx + self.offset   # 클러스터 동쪽에서 접근
        gy = 0.0                 # 복도 중앙
        yaw = math.pi           # 서쪽(클러스터 방향)을 향함

        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = center.header.frame_id or 'map'
        ps.pose.position.x = gx
        ps.pose.position.y = gy
        ps.pose.position.z = 0.0
        if self.yaw_mode == 'face_center':
            qx, qy, qz, qw = quat_from_yaw(yaw)
        else:
            qx, qy, qz, qw = (0.0, 0.0, 0.0, 1.0)
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        return ps

    def _send_goal_for_center(self, center):
        goal_pose = self._build_goal_pose(center)
        gx, gy = goal_pose.pose.position.x, goal_pose.pose.position.y
        if self._last_goal_xy is not None:
            dx = gx - self._last_goal_xy[0]
            dy = gy - self._last_goal_xy[1]
            if math.hypot(dx, dy) < self.resend_th:
                self.get_logger().info(
                    'goal too close to last goal (%.2f m < %.2f), skip resend'
                    % (math.hypot(dx, dy), self.resend_th)
                )
                return

        self.get_logger().info('waiting Nav2 action server ...')
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 action server not available!')
            return

        msg = NavigateToPose.Goal()
        msg.pose = goal_pose
        self.get_logger().info(
            'sending NavigateToPose goal -> (%.2f, %.2f) in frame %s'
            % (gx, gy, goal_pose.header.frame_id)
        )
        self._goal_in_flight = True
        self._last_goal_xy = (gx, gy)
        send_future = self._action_client.send_goal_async(msg)
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error('Nav2 rejected goal!')
            self._goal_in_flight = False
            return
        self.get_logger().info('Nav2 accepted goal, waiting result ...')
        result_future = gh.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future):
        result = future.result()
        status = result.status  # 4=SUCCEEDED, 5=CANCELED, 6=ABORTED
        status_str = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}.get(
            status, 'UNKNOWN(%d)' % status
        )
        self.get_logger().info('NavigateToPose finished: %s' % status_str)
        self._goal_in_flight = False


def main(args=None):
    rclpy.init(args=args)
    node = MineGoalSenderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
