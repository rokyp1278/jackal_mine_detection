"""
fake_mine_publisher_node.py
1단계 전용. AprilTag 가 없는 상태에서 가짜 지뢰 좌표를 직접 /mine_positions 로 발행.
또한 RViz 시각화를 위해 /mine_markers 도 같이 발행한다.

토픽:
  pub: /mine_positions   geometry_msgs/PoseArray  (frame_id = map)
  pub: /mine_markers     visualization_msgs/MarkerArray

Params:
  publish_rate: float [Hz]
  frame_id:     str   (보통 "map")
  fake_mines:   double[][]  e.g. [[2.0, 1.0, 0.0], [2.2, 1.1, 0.0], ...]
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


class FakeMinePublisherNode(Node):
    def __init__(self):
        super().__init__('fake_mine_publisher_node')

        # 파라미터 선언
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter(
            'fake_mines',
            [
                # 기본값: 군집 1 (3개) + 외톨이 1개
                2.0, 1.0, 0.0,
                2.2, 1.1, 0.0,
                2.1, 0.8, 0.0,
                5.0, -1.0, 0.0,
            ],
        )

        rate = float(self.get_parameter('publish_rate').value)
        self.frame_id = str(self.get_parameter('frame_id').value)

        # fake_mines 는 1차원 배열(rclpy 제약)으로 받아서 3개씩 끊어 사용
        flat = list(self.get_parameter('fake_mines').value)
        if len(flat) % 3 != 0:
            self.get_logger().warn(
                f'fake_mines length {len(flat)} not divisible by 3; truncating'
            )
            flat = flat[: (len(flat) // 3) * 3]
        self.points = [
            (float(flat[i]), float(flat[i + 1]), float(flat[i + 2]))
            for i in range(0, len(flat), 3)
        ]

        self.get_logger().info(
            f'Publishing {len(self.points)} fake mines on '
            f'/mine_positions at {rate} Hz (frame={self.frame_id})'
        )

        self.pose_pub = self.create_publisher(PoseArray, '/mine_positions', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/mine_markers', 10)
        self.timer = self.create_timer(1.0 / rate, self._tick)

    def _tick(self):
        now = self.get_clock().now().to_msg()

        pa = PoseArray()
        pa.header.stamp = now
        pa.header.frame_id = self.frame_id
        for (x, y, z) in self.points:
            p = Pose()
            p.position.x = x
            p.position.y = y
            p.position.z = z
            p.orientation.w = 1.0
            pa.poses.append(p)
        self.pose_pub.publish(pa)

        ma = MarkerArray()
        for i, (x, y, z) in enumerate(self.points):
            m = Marker()
            m.header.stamp = now
            m.header.frame_id = self.frame_id
            m.ns = 'fake_mines'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = z
            m.pose.orientation.w = 1.0
            m.scale.x = 0.25
            m.scale.y = 0.25
            m.scale.z = 0.25
            m.color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=0.9)
            ma.markers.append(m)
        self.marker_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = FakeMinePublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
