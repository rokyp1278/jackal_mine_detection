"""
mine_cluster_node.py
/mine_positions (PoseArray, frame=map) 를 구독해서
반경 기반 군집의 중심 좌표를 /mine_cluster_center (PoseStamped) 로 발행.
RViz 표시는 /mine_cluster_marker (Marker).

Params:
  cluster_radius:   float [m]   (default 1.0)
  min_cluster_size: int         (default 2)
  update_rate:      float [Hz]  (default 1.0)
  method:           str         "radius_count" or "dbscan"
  marker_frame:    str          보통 "map"
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

from jackal_mine_detection.utils.clustering import (
    radius_count_cluster, dbscan_cluster,
)


class MineClusterNode(Node):
    def __init__(self):
        super().__init__('mine_cluster_node')

        self.declare_parameter('cluster_radius', 1.0)
        self.declare_parameter('min_cluster_size', 2)
        self.declare_parameter('update_rate', 1.0)
        self.declare_parameter('method', 'radius_count')
        self.declare_parameter('marker_frame', 'map')

        self.radius = float(self.get_parameter('cluster_radius').value)
        self.min_count = int(self.get_parameter('min_cluster_size').value)
        self.method = str(self.get_parameter('method').value)
        self.marker_frame = str(self.get_parameter('marker_frame').value)
        rate = float(self.get_parameter('update_rate').value)

        self._latest_points = []  # list[(x,y,z)]
        self._last_center = None  # (x,y,z) or None

        self.create_subscription(
            PoseArray, '/mine_positions', self._cb_positions, 10
        )
        self.center_pub = self.create_publisher(
            PoseStamped, '/mine_cluster_center', 10
        )
        self.marker_pub = self.create_publisher(
            Marker, '/mine_cluster_marker', 10
        )
        self.timer = self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f'mine_cluster_node up. method={self.method}, '
            f'radius={self.radius}, min={self.min_count}'
        )

    def _cb_positions(self, msg: PoseArray):
        self._latest_points = [
            (p.position.x, p.position.y, p.position.z) for p in msg.poses
        ]

    def _tick(self):
        if not self._latest_points:
            return

        if self.method == 'dbscan':
            center, _members = dbscan_cluster(
                self._latest_points, eps=self.radius, min_samples=self.min_count
            )
        else:
            center, _members = radius_count_cluster(
                self._latest_points,
                radius=self.radius,
                min_count=self.min_count,
            )

        if center is None:
            return

        self._last_center = center
        now = self.get_clock().now().to_msg()

        ps = PoseStamped()
        ps.header.stamp = now
        ps.header.frame_id = self.marker_frame
        ps.pose.position.x = center[0]
        ps.pose.position.y = center[1]
        ps.pose.position.z = center[2]
        ps.pose.orientation.w = 1.0
        self.center_pub.publish(ps)

        m = Marker()
        m.header.stamp = now
        m.header.frame_id = self.marker_frame
        m.ns = 'mine_cluster'
        m.id = 0
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position.x = center[0]
        m.pose.position.y = center[1]
        m.pose.position.z = center[2]
        m.pose.orientation.w = 1.0
        m.scale.x = 2.0 * self.radius
        m.scale.y = 2.0 * self.radius
        m.scale.z = 0.05
        m.color = ColorRGBA(r=0.1, g=0.7, b=1.0, a=0.4)
        self.marker_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = MineClusterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
