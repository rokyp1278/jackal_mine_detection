"""
frontier_explorer_node.py
==========================================================================
/map (nav_msgs/OccupancyGrid) 구독 → free-unknown 경계(frontier) 자동 감지
→ 가장 유망한 frontier 군집 중심으로 NavigateToPose 순차 전송.
frontier 소진 또는 exploration_timeout_sec 경과 시 /finish_exploration 발행.

waypoint_follower_node (하드코딩 WP) 를 완전히 대체.
기존 mine_cluster / mine_goal_sender / exploration_manager 파이프라인은 그대로 사용.

동작 흐름:
  1) start_delay_sec 대기 (Nav2 완전 활성화 여유)
  2) /map 에서 frontier 군집 탐색 (BFS)
  3) score = size / (dist + 1)  가장 높은 frontier 로 NavigateToPose
  4) ABORTED 된 frontier 는 blacklist 등록 → 재시도 안 함
  5) frontier 없음이 no_frontier_limit 연속으로 감지되면 탐사 완료
  6) /finish_exploration = True 발행
==========================================================================
"""
from __future__ import annotations
import math
from collections import deque
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
import tf2_ros

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from nav2_msgs.action import NavigateToPose

from jackal_mine_detection.utils.geometry import quat_from_yaw


class FrontierExplorerNode(Node):
    FREE     =   0
    OCCUPIED = 100
    UNKNOWN  =  -1

    def __init__(self):
        super().__init__('frontier_explorer_node')

        # ── 파라미터 ─────────────────────────────────────────────────
        self.declare_parameter('nav2_action',             'navigate_to_pose')
        self.declare_parameter('min_frontier_size',       8)
        self.declare_parameter('frontier_check_rate',     0.5)
        self.declare_parameter('goal_timeout_sec',        60.0)
        self.declare_parameter('exploration_timeout_sec', 300.0)
        self.declare_parameter('no_frontier_limit',       5)
        self.declare_parameter('start_delay_sec',         5.0)
        self.declare_parameter('map_frame',               'map')
        self.declare_parameter('base_frame',              'base_link')
        self.declare_parameter('blacklist_radius_m',      0.8)
        self.declare_parameter('max_goal_dist_m',         3.0)

        action_name     = str(self.get_parameter('nav2_action').value)
        self._min_sz    = int(self.get_parameter('min_frontier_size').value)
        rate            = float(self.get_parameter('frontier_check_rate').value)
        self._g_to      = float(self.get_parameter('goal_timeout_sec').value)
        self._ex_to     = float(self.get_parameter('exploration_timeout_sec').value)
        self._no_fr_lim = int(self.get_parameter('no_frontier_limit').value)
        self._delay     = float(self.get_parameter('start_delay_sec').value)
        self._mf        = str(self.get_parameter('map_frame').value)
        self._bf        = str(self.get_parameter('base_frame').value)
        self._bl_r      = float(self.get_parameter('blacklist_radius_m').value)
        self._max_dist  = float(self.get_parameter('max_goal_dist_m').value)

        # ── 상태 ─────────────────────────────────────────────────────
        self._map:             Optional[OccupancyGrid] = None
        self._goal_in_flight:  bool  = False
        self._goal_sent_time:  Optional[float] = None
        self._goal_sent_pos:   tuple[float, float] = (0.0, 0.0)
        self._started:         bool  = False
        self._start_ts:        Optional[float] = None
        self._explore_ts:      Optional[float] = None
        self._no_fr_count:     int   = 0
        self._finished:        bool  = False
        self._blacklist:       list[tuple[float, float]] = []

        # ── TF ───────────────────────────────────────────────────────
        self._tf_buf = tf2_ros.Buffer()
        self._tf_lis = tf2_ros.TransformListener(self._tf_buf, self)

        # ── Sub / Pub / Action ───────────────────────────────────────
        self.create_subscription(OccupancyGrid, '/map', self._cb_map, 1)
        self._pub_fin = self.create_publisher(Bool, '/finish_exploration', 10)
        self._ac      = ActionClient(self, NavigateToPose, action_name)

        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f'frontier_explorer 시작 대기 중 (delay={self._delay}s) | '
            f'min_size={self._min_sz}cells, blacklist_r={self._bl_r}m'
        )

    # ─────────────────────────────────────────────────────────────────
    def _cb_map(self, msg: OccupancyGrid):
        self._map = msg

    # ─────────────────────────────────────────────────────────────────
    def _tick(self):
        if self._finished:
            return

        now = self.get_clock().now().nanoseconds / 1e9

        # 시작 딜레이
        if not self._started:
            if self._start_ts is None:
                self._start_ts = now
            if now - self._start_ts < self._delay:
                return
            self._started    = True
            self._explore_ts = now
            self.get_logger().info('▶ frontier 탐사 시작!')

        # 전체 타임아웃
        if now - self._explore_ts > self._ex_to:
            self.get_logger().warn('탐사 타임아웃 → /finish_exploration 발행')
            self._finish()
            return

        # goal 타임아웃 (Nav2 무응답 안전장치)
        if self._goal_in_flight and self._goal_sent_time is not None:
            if now - self._goal_sent_time > self._g_to:
                self.get_logger().warn('goal 타임아웃, blacklist 후 재탐색')
                self._blacklist.append(self._goal_sent_pos)
                self._goal_in_flight = False

        if self._goal_in_flight or self._map is None:
            return

        # frontier 탐색
        frontiers = self._find_frontier_clusters()
        valid = [f for f in frontiers if not self._is_blacklisted(f['cx'], f['cy'])]

        if not valid:
            self._no_fr_count += 1
            self.get_logger().info(
                f'유효 frontier 없음 ({self._no_fr_count}/{self._no_fr_lim})'
            )
            if self._no_fr_count >= self._no_fr_lim:
                self.get_logger().info('탐사 완료 → /finish_exploration 발행')
                self._finish()
            return

        self._no_fr_count = 0

        # 로봇 위치 취득
        rx, ry = self._robot_xy()

        # score = size / (dist_to_frontier + 1)  →  크고 가까운 frontier 우선
        def score(f):
            dist = math.hypot(f['cx'] - rx, f['cy'] - ry) if rx is not None else 1.0
            return f['size'] / (dist + 1.0)

        best = max(valid, key=score)
        gx, gy = best['cx'], best['cy']

        # frontier가 너무 멀면 중간 지점만 이동 (단계적 탐사로 탐지 기회 확보)
        if rx is not None:
            dist = math.hypot(gx - rx, gy - ry)
            if dist > self._max_dist:
                ratio = self._max_dist / dist
                gx = rx + (gx - rx) * ratio
                gy = ry + (gy - ry) * ratio

        self._send_goal(gx, gy, rx, ry)

    # ─────────────────────────────────────────────────────────────────
    def _find_frontier_clusters(self) -> list[dict]:
        """
        BFS 기반 frontier 군집 탐색.
        frontier 셀 = FREE(0) 이면서 UNKNOWN(-1) 인접 셀을 가진 셀.
        군집화 후 world 좌표 중심 + 셀 수 반환.
        """
        m   = self._map
        w   = m.info.width
        h   = m.info.height
        res = m.info.resolution
        ox  = m.info.origin.position.x
        oy  = m.info.origin.position.y
        data = m.data  # int8[] : 0=free, 100=occ, -1=unknown

        def cell(x, y): return y * w + x

        # 1) frontier 마킹
        is_frontier = bytearray(w * h)
        for y in range(1, h - 1):
            for x in range(1, w - 1):
                if data[cell(x, y)] != self.FREE:
                    continue
                for dx, dy in ((1,0),(-1,0),(0,1),(0,-1)):
                    if data[cell(x+dx, y+dy)] == self.UNKNOWN:
                        is_frontier[cell(x, y)] = 1
                        break

        # 2) BFS 군집화
        visited  = bytearray(w * h)
        clusters = []
        for y in range(h):
            for x in range(w):
                i = cell(x, y)
                if not is_frontier[i] or visited[i]:
                    continue

                q     = deque([(x, y)])
                cells = []
                visited[i] = 1
                while q:
                    cx, cy = q.popleft()
                    cells.append((cx, cy))
                    for dx, dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(-1,1),(1,-1),(-1,-1)):
                        nx, ny = cx+dx, cy+dy
                        if 0 <= nx < w and 0 <= ny < h:
                            ni = cell(nx, ny)
                            if is_frontier[ni] and not visited[ni]:
                                visited[ni] = 1
                                q.append((nx, ny))

                if len(cells) >= self._min_sz:
                    avg_cx = sum(c[0] for c in cells) / len(cells)
                    avg_cy = sum(c[1] for c in cells) / len(cells)
                    clusters.append({
                        'cx':   ox + (avg_cx + 0.5) * res,
                        'cy':   oy + (avg_cy + 0.5) * res,
                        'size': len(cells),
                    })

        return clusters

    # ─────────────────────────────────────────────────────────────────
    def _robot_xy(self) -> tuple[Optional[float], Optional[float]]:
        try:
            tf = self._tf_buf.lookup_transform(
                self._mf, self._bf,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )
            return tf.transform.translation.x, tf.transform.translation.y
        except Exception:
            return None, None

    def _is_blacklisted(self, gx: float, gy: float) -> bool:
        return any(math.hypot(gx-bx, gy-by) < self._bl_r
                   for bx, by in self._blacklist)

    # ─────────────────────────────────────────────────────────────────
    def _send_goal(self, gx: float, gy: float,
                   rx: Optional[float], ry: Optional[float]):
        if not self._ac.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server 없음')
            return

        # 로봇 → frontier 방향으로 yaw 설정
        if rx is not None:
            yaw = math.atan2(gy - ry, gx - rx)
        else:
            yaw = 0.0

        ps = PoseStamped()
        ps.header.stamp    = self.get_clock().now().to_msg()
        ps.header.frame_id = self._mf
        ps.pose.position.x = gx
        ps.pose.position.y = gy
        ps.pose.position.z = 0.0
        qx, qy, qz, qw    = quat_from_yaw(yaw)
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw

        goal = NavigateToPose.Goal()
        goal.pose = ps

        self._goal_in_flight = True
        self._goal_sent_time = self.get_clock().now().nanoseconds / 1e9
        self._goal_sent_pos  = (gx, gy)

        self.get_logger().info(
            f'▷ frontier goal → ({gx:.2f}, {gy:.2f}), '
            f'yaw={math.degrees(yaw):.0f}°'
        )
        fut = self._ac.send_goal_async(goal)
        fut.add_done_callback(self._on_response)

    def _on_response(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().warn('Nav2 goal rejected')
            self._blacklist.append(self._goal_sent_pos)
            self._goal_in_flight = False
            return
        gh.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        status     = future.result().status
        status_str = {4:'SUCCEEDED', 5:'CANCELED', 6:'ABORTED'}.get(
            status, f'UNKNOWN({status})'
        )
        gx, gy = self._goal_sent_pos
        self.get_logger().info(
            f'frontier goal {status_str}: ({gx:.2f}, {gy:.2f})'
        )
        if status == 6:  # ABORTED
            self._blacklist.append((gx, gy))
            self.get_logger().warn(f'blacklist 추가: ({gx:.2f}, {gy:.2f})')
        self._goal_in_flight = False

    # ─────────────────────────────────────────────────────────────────
    def _finish(self):
        if self._finished:
            return
        self._finished = True
        msg = Bool()
        msg.data = True
        self._pub_fin.publish(msg)
        self.get_logger().info('탐사 종료!')


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
