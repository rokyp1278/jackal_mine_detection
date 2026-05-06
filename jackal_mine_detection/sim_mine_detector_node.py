"""
sim_mine_detector_node.py
======================================================================
WSL 환경에서 Gazebo 카메라 렌더링이 불가할 때 사용하는 대체 탐지 노드.

카메라/AprilTag 파이프라인을 완전히 건너뛰고,
로봇 TF 위치만으로 지뢰(태그) 탐지를 시뮬레이션한다.

조건:
  1) 로봇-태그 거리 < detect_range_m
  2) 태그가 카메라 FOV(±fov_half_deg) 안에 있을 때
→ 해당 태그를 탐지 목록에 추가
→ /mine_positions (PoseArray) + /mine_markers (MarkerArray) 발행

이 토픽들은 mine_cluster_node 가 구독하므로 기존 파이프라인 나머지는 그대로 동작.

파라미터:
  tag_positions:  float[]  flat: x0,y0,z0, x1,y1,z1, ...  (world 좌표)
  tag_ids:        int[]    각 태그 ID
  detect_range_m: float    탐지 거리 [m]  (default 2.0)
  fov_half_deg:   float    카메라 반-FOV [deg] (default 45.0)
  publish_rate:   float    [Hz] (default 5.0)
  map_frame:      str      (default "map")
  base_frame:     str      (default "base_link")
======================================================================
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import tf2_ros

from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


class SimMineDetectorNode(Node):
    def __init__(self):
        super().__init__('sim_mine_detector_node')

        # ── 파라미터 ─────────────────────────────────────────────────
        self.declare_parameter('detect_range_m', 2.0)
        self.declare_parameter('fov_half_deg',   45.0)
        self.declare_parameter('publish_rate',   5.0)
        self.declare_parameter('map_frame',  'map')
        self.declare_parameter('base_frame', 'base_link')
        # world 파일의 태그 위치 (mine_detection.world 와 일치)
        self.declare_parameter('tag_positions', [
            3.0,  1.45, 0.8,   # Tag 0  북쪽 벽
            3.3,  1.45, 0.8,   # Tag 1  북쪽 벽
            2.7,  1.45, 0.8,   # Tag 2  북쪽 벽
            8.0, -1.45, 0.8,   # Tag 3  남쪽 벽
        ])
        self.declare_parameter('tag_ids', [0, 1, 2, 3])

        self._range    = float(self.get_parameter('detect_range_m').value)
        self._fov_half = math.radians(float(self.get_parameter('fov_half_deg').value))
        rate           = float(self.get_parameter('publish_rate').value)
        self._map_f    = str(self.get_parameter('map_frame').value)
        self._base_f   = str(self.get_parameter('base_frame').value)

        flat = list(self.get_parameter('tag_positions').value)
        ids  = list(self.get_parameter('tag_ids').value)
        self._tags = [
            (int(ids[i]), float(flat[i*3]), float(flat[i*3+1]), float(flat[i*3+2]))
            for i in range(len(ids))
        ]

        # 탐지된 태그: {tag_id: (x, y, z)}
        self._detected: dict[int, tuple[float, float, float]] = {}

        # ── TF2 ─────────────────────────────────────────────────────
        self._tf_buf = tf2_ros.Buffer()
        self._tf_lis = tf2_ros.TransformListener(self._tf_buf, self)

        # ── Publisher ────────────────────────────────────────────────
        self._pub_pos    = self.create_publisher(PoseArray,   '/mine_positions', 10)
        self._pub_marker = self.create_publisher(MarkerArray, '/mine_markers',   10)

        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f'sim_mine_detector 시작. '
            f'range={self._range}m, fov=±{math.degrees(self._fov_half):.0f}°, '
            f'태그 {len(self._tags)}개 등록'
        )

    # ─────────────────────────────────────────────────────────────────
    def _tick(self):
        # 로봇 위치/방향 취득 (map → base_link)
        try:
            tf = self._tf_buf.lookup_transform(
                self._map_f, self._base_f,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return

        rx = tf.transform.translation.x
        ry = tf.transform.translation.y
        q  = tf.transform.rotation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        robot_yaw = math.atan2(siny, cosy)

        # 각 태그 탐지 확인
        for (tid, tx, ty, tz) in self._tags:
            if tid in self._detected:
                continue  # 이미 탐지됨

            dx   = tx - rx
            dy   = ty - ry
            dist = math.hypot(dx, dy)

            if dist > self._range:
                continue

            # 태그가 카메라 FOV 안에 있는지 확인
            angle_to_tag = math.atan2(dy, dx)
            diff = self._norm(angle_to_tag - robot_yaw)
            if abs(diff) > self._fov_half:
                continue

            # 새 탐지 등록
            self._detected[tid] = (tx, ty, tz)
            self.get_logger().info(
                f'[sim_mine_detector] ★ Tag {tid} 탐지! '
                f'위치=({tx:.2f}, {ty:.2f}), 거리={dist:.1f}m'
            )

        # 탐지된 모든 지뢰 발행
        self._publish(rx, ry)

    # ─────────────────────────────────────────────────────────────────
    def _publish(self, rx: float, ry: float):
        if not self._detected:
            return

        now     = self.get_clock().now().to_msg()
        entries = list(self._detected.items())   # [(tag_id, (x,y,z)), ...]

        # ── PoseArray (/mine_positions) ───────────────────────────────
        pa = PoseArray()
        pa.header.stamp    = now
        pa.header.frame_id = self._map_f
        for _, (x, y, z) in entries:
            p = Pose()
            p.position.x = x
            p.position.y = y
            p.position.z = z
            p.orientation.w = 1.0
            pa.poses.append(p)
        self._pub_pos.publish(pa)

        # ── MarkerArray (/mine_markers) ───────────────────────────────
        ma = MarkerArray()
        for i, (tid, (x, y, z)) in enumerate(entries):
            m = Marker()
            m.header.stamp    = now
            m.header.frame_id = self._map_f
            m.ns              = 'sim_mines'
            m.id              = i
            m.type            = Marker.CYLINDER
            m.action          = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = z / 2.0   # 바닥부터 높이
            m.pose.orientation.w = 1.0
            m.scale.x = 0.15
            m.scale.y = 0.15
            m.scale.z = z                  # 태그 높이까지
            m.color   = ColorRGBA(r=1.0, g=0.4, b=0.0, a=0.9)  # 주황
            ma.markers.append(m)
        self._pub_marker.publish(ma)

    @staticmethod
    def _norm(a: float) -> float:
        while a >  math.pi: a -= 2 * math.pi
        while a < -math.pi: a += 2 * math.pi
        return a


def main(args=None):
    rclpy.init(args=args)
    node = SimMineDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
