import math

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from turtlesim.msg import Pose


PARAM_RANGES = {
    'linear_speed': (0.0, 1.0),
    'turn_rate': (-1.0, 1.0),
    'publish_hz': (1.0, 30.0),
}


def validate_param(name, value):
    """
    Check whether a parameter value is admissible.

    Returns (ok, reason).
    """
    if name not in PARAM_RANGES:
        return False, f"unknown parameter '{name}'"
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False, f"{name} must be a number, got {type(value).__name__}"
    if not math.isfinite(value):
        return False, f"{name} must be finite, got {value}"
    lo, hi = PARAM_RANGES[name]
    if not (lo <= value <= hi):
        return False, f"{name} must be in [{lo}, {hi}], got {value}"
    return True, ""


def compute_command(pose, linear_speed, turn_rate):
    """
    Pure function: turn the last pose into a Twist.

    Without a pose returns zero. Otherwise uses given speed and turn rate.
    """
    cmd = Twist()
    if pose is None:
        return cmd
    cmd.linear.x = float(linear_speed)
    cmd.angular.z = float(turn_rate)
    return cmd


class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol')

        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('turn_rate', 0.3)
        self.declare_parameter('publish_hz', 10.0)

        self.pose_subscription = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10)
        self.last_pose = None

        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.timer = None
        self._recreate_timer(self.get_parameter('publish_hz').value)

        self.add_on_set_parameters_callback(self._on_set_parameters)

    def _recreate_timer(self, hz):
        if self.timer is not None:
            self.timer.cancel()
            self.destroy_timer(self.timer)
            self.timer = None
        self.timer = self.create_timer(1.0 / hz, self.timer_callback)
        self.get_logger().info(
            f"timer period = {1.0 / hz:.3f} s ({hz} Hz)")

    def _on_set_parameters(self, params):
        # Сначала валидация всех параметров без побочных эффектов.
        '''for p in params:
            ok, reason = validate_param(p.name, p.value)
            if not ok:
                self.get_logger().warn(
                    f"rejected {p.name}={p.value}: {reason}")
                return SetParametersResult(successful=False, reason=reason)
'''
        # Все значения допустимы — применяем изменения.
        for p in params:
            if p.name == 'publish_hz':
                current = float(self.get_parameter('publish_hz').value)
                if float(p.value) != current:
                    self._recreate_timer(float(p.value))

        return SetParametersResult(successful=True)
    def pose_callback(self, msg: Pose):
        """Store the last pose."""
        self.last_pose = msg

    def timer_callback(self):
        """Publish a Twist built from the current parameters."""
        linear = float(self.get_parameter('linear_speed').value)
        turn = float(self.get_parameter('turn_rate').value)
        cmd = compute_command(self.last_pose, linear, turn)
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
