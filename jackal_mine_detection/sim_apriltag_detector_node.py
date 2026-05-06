"""
sim_apriltag_detector_node.py
======================================================================
WSL 환경에서 Gazebo 카메라 센서가 OpenGL 렌더링 문제로 동작하지 않을 때
로봇 TF 위치 기반으로 AprilTag 탐지를 시뮬레이션하는 대체 노드.

동작 원리:
  1) map → base_link TF 로 로봇 현재 위치/방향 파악
  2) 각 태그까지의 거리와 각도 계산
  3) 거리 < detect_range_m  AND  카메라 FOV 안에 있으면
     → AprilTagDetectionArray 를 /detections 에 발행
     → 태그 pose 는 map 좌표계로 발행 (mine_recorder 에서 map→map 변환 = 무변환)

파라미터:
  tag_positions: float[]  flat: x0,y0,z0, x1,y1,z1, ...   (world 좌표)
  tag_ids:       int[]    각 태그 ID
  detect_range_m: float   탐지 거리 [m]  (default 2.0)
  fov_half_deg:   float   카메라 반-FOV [deg] (default 45.0)
  publish_rate:   float   [Hz]  (default 10.0)
  map_frame:      str     (default "map")
  base_frame:     str     (default "base_link")
======================================================================
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import tf2_ros

from apriltag_msgs.msg import AprilTagDetectionArray, AprilTagDetection


class SimAprilTagDetectorNode(Node):
    def __init__(self):
        super().__init__('sim_apriltag_detector_node')

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

        # ── TF2 ─────────────────────────────────────────────────────
        self._tf_buf = tf2_ros.Buffer()
        self._tf_lis = tf2_ros.TransformListener(self._tf_buf, self)

        # ── Publisher ────────────────────────────────────────────────
        self._pub = self.create_publisher(
            AprilTagDetectionArray, '/detections', 10
        )

        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f'sim_apriltag_detector 시작. '
            f'range={self._range}m, fov={math.degrees(self._fov_half)*2:.0f}°, '
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

        # quaternion → yaw
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        robot_yaw = math.atan2(siny, cosy)

        now = self.get_clock().now()
        arr = AprilTagDetectionArray()
        arr.header.stamp    = now.to_msg()
        arr.header.frame_id = self._map_f

        for (tid, tx, ty, tz) in self._tags:
            dx   = tx - rx
            dy   = ty - ry
            dist = math.hypot(dx, dy)

            if dist > self._range:
                continue

            # 태그 방향과 로봇 진행 방향의 각도 차
            angle_to_tag = math.atan2(dy, dx)
            diff = self._norm(angle_to_tag - robot_yaw)
            if abs(diff) > self._fov_half:
                continue

            # ── 탐지 메시지 ───────────────────────────────────────────
            det = AprilTagDetection()
            det.family          = '36h11'
            det.id              = tid
            det.hamming         = 0
            det.decision_margin = 100.0

            # pose 를 map 좌표계로 발행
            # → mine_recorder_node 에서 map→map 변환 = 무변환 → 정확한 위치 저장
            det.pose.header.stamp    = now.to_msg()
            det.pose.header.frame_id = self._map_f   # map 좌표계 명시
            det.pose.pose.pose.position.x    = tx
            det.pose.pose.pose.position.y    = ty
            det.pose.pose.pose.position.z    = tz
            det.pose.pose.pose.orientation.w = 1.0

            arr.detections.append(det)

        if arr.detections:
            self.get_logger().info(
                f'탐지: ' + ', '.join(f'tag{d.id}(거리{math.hypot(self._tags[d.id][1]-rx, self._tags[d.id][2]-ry):.1f}m)' for d in arr.detections),
                throttle_duration_sec=2.0,
            )

        self._pub.publish(arr)

    @staticmethod
    def _norm(a: float) -> float:
        while a >  math.pi: a -= 2 * math.pi
        while a < -math.pi: a += 2 * math.pi
        return a


def main(args=None):
    rclpy.init(args=args)
    node = SimAprilTagDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
