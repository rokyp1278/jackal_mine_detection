"""
exploration_manager_node.py
미션 상태 머신.
- 시작 시 EXPLORING 상태
- /finish_exploration 토픽 (std_msgs/Bool) 으로 true 가 들어오거나
- exploration_timeout_sec 초과 시 FINISHED 로 전이 후
- /trigger_final_goal 토픽 (std_msgs/Empty) 발행 -> mine_goal_sender_node 트리거
- /exploration_state (std_msgs/String) 으로 현재 상태 발행

Params:
  exploration_timeout_sec: 300.0
  finish_topic: "/finish_exploration"
  state_topic:  "/exploration_state"
  trigger_topic:"/trigger_final_goal"
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, String


STATE_EXPLORING = 'EXPLORING'
STATE_FINISHED = 'FINISHED'
STATE_GOAL_SENT = 'GOAL_SENT'
STATE_DONE = 'DONE'


class ExplorationManagerNode(Node):
    def __init__(self):
        super().__init__('exploration_manager_node')

        self.declare_parameter('exploration_timeout_sec', 300.0)
        self.declare_parameter('finish_topic', '/finish_exploration')
        self.declare_parameter('state_topic', '/exploration_state')
        self.declare_parameter('trigger_topic', '/trigger_final_goal')
        self.declare_parameter('publish_rate', 1.0)

        self.timeout = float(self.get_parameter('exploration_timeout_sec').value)
        finish_topic = str(self.get_parameter('finish_topic').value)
        state_topic = str(self.get_parameter('state_topic').value)
        trigger_topic = str(self.get_parameter('trigger_topic').value)
        rate = float(self.get_parameter('publish_rate').value)

        self.state = STATE_EXPLORING
        self.start_time = self.get_clock().now()

        self.create_subscription(Bool, finish_topic, self._cb_finish, 10)
        self.state_pub = self.create_publisher(String, state_topic, 10)
        self.trigger_pub = self.create_publisher(Empty, trigger_topic, 10)
        self.timer = self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f'exploration_manager_node up. timeout={self.timeout}s, '
            f'state_topic={state_topic}, trigger_topic={trigger_topic}'
        )

    def _cb_finish(self, msg: Bool):
        if msg.data and self.state == STATE_EXPLORING:
            self.get_logger().info('Got /finish_exploration=true -> FINISHED')
            self.state = STATE_FINISHED

    def _tick(self):
        # 타임아웃 체크
        if self.state == STATE_EXPLORING:
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            if elapsed >= self.timeout:
                self.get_logger().info(
                    f'Exploration timeout {elapsed:.1f}s >= {self.timeout}s -> FINISHED'
                )
                self.state = STATE_FINISHED

        # FINISHED -> trigger 보내고 GOAL_SENT 로
        if self.state == STATE_FINISHED:
            self.get_logger().info('Publishing /trigger_final_goal')
            self.trigger_pub.publish(Empty())
            self.state = STATE_GOAL_SENT

        # 상태 publish
        s = String()
        s.data = self.state
        self.state_pub.publish(s)


def main(args=None):
    rclpy.init(args=args)
    node = ExplorationManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
