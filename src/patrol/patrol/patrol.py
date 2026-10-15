from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose


def compute_command(pose):
    """
    Чистая функция: по последней позе возвращает Twist.

    Если pose is None — нулевая команда.
    """
    cmd = Twist()
    if pose is None:
        return cmd

    cmd.linear.x = 0.5
    cmd.angular.z = 0.3
    return cmd


class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol')

        # Подписка на позу (поле объекта — требование ТЗ)
        self.pose_subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10,
        )
        self.last_pose = None

        # Издатель в ОТНОСИТЕЛЬНЫЙ топик 'cmd_vel' (дефект для стадии 2)
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Таймер 0.1 с
        self.timer = self.create_timer(0.1, self.timer_callback)

    def pose_callback(self, msg: Pose):
        """Сохрани последнее сообщение."""
        self.last_pose = msg

    def timer_callback(self):
        """Таймер публикует Twist, вычисленный чистой функцией."""
        cmd = compute_command(self.last_pose)
        self.cmd_publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
