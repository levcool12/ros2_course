import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from rcl_interfaces.msg import SetParametersResult

PARAM_RANGES = {
    'linear_speed': (0.0, 1.0),
    'turn_rate': (-1.0, 1.0),
    'publish_hz': (1.0, 30.0),
}

def validate_param(name, value):
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
    cmd = Twist()
    if pose is None:
        return cmd
    cmd.linear.x = linear_speed
    cmd.angular.z = turn_rate
    return cmd

class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol')
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('turn_rate', 0.3)
        self.declare_parameter('publish_hz', 10.0)

        self.linear_speed = self.get_parameter('linear_speed').value
        self.turn_rate = self.get_parameter('turn_rate').value
        self.publish_hz = self.get_parameter('publish_hz').value

        self.pose_subscription = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.last_pose = None
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(1.0 / self.publish_hz, self.timer_callback)
        self.add_on_set_parameters_callback(self.parameters_callback)

    def pose_callback(self, msg: Pose):
        self.last_pose = msg

    def timer_callback(self):
        cmd = compute_command(self.last_pose, self.linear_speed, self.turn_rate)
        self.cmd_publisher.publish(cmd)

    def parameters_callback(self, params):
        for param in params:
            ok, reason = validate_param(param.name, param.value)
            if not ok:
                return SetParametersResult(successful=False, reason=reason)

        for param in params:
            if param.name == 'linear_speed':
                self.linear_speed = param.value
            elif param.name == 'turn_rate':
                self.turn_rate = param.value
            elif param.name == 'publish_hz':
                if self.publish_hz != param.value:
                    self.publish_hz = param.value
                    self.timer.cancel()
                    self.timer = self.create_timer(1.0 / self.publish_hz, self.timer_callback)
                    self.get_logger().info(f'Timer recreated for {self.publish_hz} Hz')

        return SetParametersResult(successful=True)

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
