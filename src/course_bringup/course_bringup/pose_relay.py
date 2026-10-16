from geometry_msgs.msg import Twist  # noqa: F401 (оставим, если пригодится)
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy,
)
from turtlesim.msg import Pose


def make_qos(reliability: str) -> QoSProfile:
    """Build a QoSProfile from a reliability name."""
    rel = (ReliabilityPolicy.RELIABLE
           if reliability == 'reliable'
           else ReliabilityPolicy.BEST_EFFORT)
    return QoSProfile(
        reliability=rel,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


class PoseRelay(Node):
    def __init__(self):
        super().__init__('pose_relay')

        self.declare_parameter('subscriber_reliability', 'best_effort')
        rel_name = self.get_parameter('subscriber_reliability').value

        # Publisher: sensor stream, best_effort (см. ТЗ)
        self.publisher = self.create_publisher(
            Pose, 'sensor_pose', make_qos('best_effort'))

        # Подписка на turtlesim (глобальный топик, абсолютное имя)
        self.pose_sub = self.create_subscription(
            Pose, '/turtle1/pose', self._on_pose, make_qos('best_effort'))

        # Отдельный подписчик с настраиваемым QoS — для диагностики
        self.sensor_sub = self.create_subscription(
            Pose, 'sensor_pose', self._on_sensor, make_qos(rel_name))

        self.get_logger().info(
            f"pose_relay up, subscriber_reliability={rel_name}")

    def _on_pose(self, msg: Pose):
        self.publisher.publish(msg)

    def _on_sensor(self, msg: Pose):
        self.get_logger().info(
            f"received sensor_pose x={msg.x:.3f} y={msg.y:.3f}")


def main(args=None):
    rclpy.init(args=args)
    node = PoseRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
