#!/usr/bin/env python3
"""
mine_recorder_node.py  (2단계 완전 구현)

AprilTag detection 을 map frame 으로 변환하여 저장한다.

구독:
  /detections  apriltag_msgs/AprilTagDetectionArray
  /tf, /tf_static  (TF2 Buffer)

발행:
  /mine_positions   geometry_msgs/PoseArray       (frame=map)
  /mine_markers     visualization_msgs/MarkerArray

파라미터:
  map_frame:           str   "map"
  camera_frame:        str   "camera_color_optical_frame"
  duplicate_distance:  float  0.3   [m]  거리 기반 중복 제거
  use_tag_id_dedup:    bool   true   tag_id 기반 중복 제거 우선
  csv_path:            str   "/tmp/mine_positions.csv"
  marker_scale:        float  0.2
  marker_lifetime_sec: float  0.0   (0 = 영구)
  tf_timeout_sec:      float  0.5
  publish_rate:        float  2.0   [Hz]
"""

import os
import csv
import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA

import tf2_ros
import tf2_geometry_msgs  # noqa: F401  PoseStamped transform 등록에 필요

from apriltag_msgs.msg import AprilTagDetectionArray


class MineRecorderNode(Node):
    def __init__(self):
        super().__init__('mine_recorder_node')

        # ── 파라미터 선언 ──────────────────────────────────────────────
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('duplicate_distance', 0.3)
        self.declare_parameter('use_tag_id_dedup', True)
        self.declare_parameter('csv_path', '/tmp/mine_positions.csv')
        self.declare_parameter('marker_scale', 0.2)
        self.declare_parameter('marker_lifetime_sec', 0.0)
        self.declare_parameter('tf_timeout_sec', 0.5)
        self.declare_parameter('publish_rate', 2.0)

        self.map_frame      = str(self.get_parameter('map_frame').value)
        self.camera_frame   = str(self.get_parameter('camera_frame').value)
        self.dup_dist       = float(self.get_parameter('duplicate_distance').value)
        self.use_id_dedup   = bool(self.get_parameter('use_tag_id_dedup').value)
        self.csv_path       = str(self.get_parameter('csv_path').value)
        self.marker_scale   = float(self.get_parameter('marker_scale').value)
        self.marker_lifetime= float(self.get_parameter('marker_lifetime_sec').value)
        self.tf_timeout     = float(self.get_parameter('tf_timeout_sec').value)
        rate                = float(self.get_parameter('publish_rate').value)

        # ── TF2 ───────────────────────────────────────────────────────
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── 저장소 ────────────────────────────────────────────────────
        # tag_id(int) -> (x, y, z)   id 기반 dedup 용
        self._id_db: dict[int, tuple[float, float, float]] = {}
        # [(x, y, z), ...]            거리 기반 dedup 용
        self._dist_db: list[tuple[float, float, float]] = []

        # ── CSV 초기화 ────────────────────────────────────────────────
        self._init_csv()

        # ── Publisher ─────────────────────────────────────────────────
        self._pub_pos    = self.create_publisher(PoseArray,   '/mine_positions', 10)
        self._pub_marker = self.create_publisher(MarkerArray, '/mine_markers',   10)

        # ── Subscriber ────────────────────────────────────────────────
        self.create_subscription(
            AprilTagDetectionArray, '/detections', self._cb_detections, 10
        )

        # ── 발행 타이머 ────────────────────────────────────────────────
        self.create_timer(1.0 / rate, self._publish_all)

        self.get_logger().info(
            f'mine_recorder_node 시작. map={self.map_frame}, '
            f'camera={self.camera_frame}, '
            f'id_dedup={self.use_id_dedup}, dup_dist={self.dup_dist}m'
        )

    # ─────────────────────────────────────────────────────────────────
    def _init_csv(self):
        try:
            parent = os.path.dirname(os.path.abspath(self.csv_path))
            os.makedirs(parent, exist_ok=True)
            with open(self.csv_path, 'w', newline='') as f:
                csv.writer(f).writerow(['tag_id', 'x', 'y', 'z', 'timestamp_sec'])
            self.get_logger().info(f'CSV 초기화: {self.csv_path}')
        except Exception as e:
            self.get_logger().error(f'CSV 초기화 실패: {e}')

    # ─────────────────────────────────────────────────────────────────
    def _cb_detections(self, msg: AprilTagDetectionArray):
        for det in msg.detections:
            # apriltag_msgs ROS2 Humble: id 는 int32 (단일 정수)
            # int32[] 배열 형식도 방어적으로 처리
            raw_id = det.id
            if isinstance(raw_id, (list, tuple, bytes)):
                if len(raw_id) == 0:
                    continue
                tag_id = int(raw_id[0])
            else:
                tag_id = int(raw_id)

            # tag_id 기반 중복 제거
            if self.use_id_dedup and tag_id in self._id_db:
                continue

            # PoseStamped (camera frame) 구성
            pose_cam = PoseStamped()
            pose_cam.header = det.pose.header
            if not pose_cam.header.frame_id:
                pose_cam.header.frame_id = self.camera_frame
            # PoseWithCovarianceStamped → PoseWithCovariance → Pose
            pose_cam.pose = det.pose.pose.pose

            # map frame 으로 TF 변환
            try:
                pose_map = self.tf_buffer.transform(
                    pose_cam,
                    self.map_frame,
                    timeout=Duration(seconds=self.tf_timeout),
                )
            except tf2_ros.LookupException as e:
                self.get_logger().warn(
                    f'TF LookupException (tag {tag_id}): {e}',
                    throttle_duration_sec=3.0,
                )
                continue
            except tf2_ros.ExtrapolationException as e:
                self.get_logger().warn(
                    f'TF ExtrapolationException (tag {tag_id}): {e}',
                    throttle_duration_sec=3.0,
                )
                continue
            except Exception as e:
                self.get_logger().warn(
                    f'TF 변환 오류 (tag {tag_id}): {e}',
                    throttle_duration_sec=3.0,
                )
                continue

            x = pose_map.pose.position.x
            y = pose_map.pose.position.y
            z = pose_map.pose.position.z

            # 거리 기반 중복 제거 (use_id_dedup=False 일 때)
            if not self.use_id_dedup:
                too_close = any(
                    math.hypot(x - ex, y - ey) < self.dup_dist
                    for ex, ey, _ in self._dist_db
                )
                if too_close:
                    continue
                self._dist_db.append((x, y, z))

            # 저장
            self._id_db[tag_id] = (x, y, z)

            # CSV 기록
            self._append_csv(tag_id, x, y, z)

            self.get_logger().info(
                f'[mine_recorder] tag {tag_id} 저장 → map ({x:.3f}, {y:.3f}, {z:.3f})'
            )

    # ─────────────────────────────────────────────────────────────────
    def _append_csv(self, tag_id: int, x: float, y: float, z: float):
        try:
            with open(self.csv_path, 'a', newline='') as f:
                csv.writer(f).writerow([
                    tag_id,
                    round(x, 4), round(y, 4), round(z, 4),
                    self.get_clock().now().nanoseconds // 10 ** 9,
                ])
        except Exception as e:
            self.get_logger().error(f'CSV 기록 실패: {e}')

    # ─────────────────────────────────────────────────────────────────
    def _publish_all(self):
        entries: list[tuple[float, float, float]] = (
            list(self._id_db.values()) if self.use_id_dedup else self._dist_db
        )
        if not entries:
            return

        now = self.get_clock().now().to_msg()

        # ── PoseArray ─────────────────────────────────────────────────
        pa = PoseArray()
        pa.header.stamp    = now
        pa.header.frame_id = self.map_frame
        for x, y, z in entries:
            p = Pose()
            p.position.x = x
            p.position.y = y
            p.position.z = z
            p.orientation.w = 1.0
            pa.poses.append(p)
        self._pub_pos.publish(pa)

        # ── MarkerArray ───────────────────────────────────────────────
        ma = MarkerArray()
        lt_sec  = int(self.marker_lifetime)
        lt_nsec = int((self.marker_lifetime - lt_sec) * 1e9)
        for i, (x, y, z) in enumerate(entries):
            m = Marker()
            m.header.stamp    = now
            m.header.frame_id = self.map_frame
            m.ns              = 'mine_positions'
            m.id              = i
            m.type            = Marker.CUBE
            m.action          = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = z
            m.pose.orientation.w = 1.0
            m.scale.x = self.marker_scale
            m.scale.y = self.marker_scale
            m.scale.z = self.marker_scale
            m.color   = ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0)
            if self.marker_lifetime > 0.0:
                m.lifetime.sec     = lt_sec
                m.lifetime.nanosec = lt_nsec
            ma.markers.append(m)
        self._pub_marker.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = MineRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
