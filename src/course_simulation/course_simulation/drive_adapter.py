"""PR07 drive adapter: /drive/safe (AckermannDriveStamped) ->
/vehicle/gz_cmd_vel (Twist) for the Gazebo AckermannSteering system.

Contract (contracts/topics.md):
  drive.speed          -> v = clamp(speed, 0.0, 1.2) m/s,  linear.x = v
  drive.steering_angle -> d = clamp(angle, -0.45, 0.45) rad,
                          angular.z = v * tan(d) / L,  L = 0.8 m
  NaN/Inf in any field or header.stamp older than 0.25 s
                       -> zero Twist + diagnostic reason,
                          the previous command is NOT repeated.

The node runs with use_sim_time:=true; "now" is therefore simulation
time taken from /clock. The pure function below keeps the conversion
unit-testable without a running simulation.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist

# Vehicle contract constants.
MAX_SPEED = 1.2            # m/s
MAX_STEER_ANGLE = 0.45     # rad
WHEEL_BASE = 0.8           # m, matches course_description (PR06)
MAX_STAMP_AGE = 0.25       # s


def compute_twist(drive: AckermannDriveStamped, now_sec: float):
    """Convert one Ackermann command into a Twist.

    Returns (twist, reason): reason is None for a valid command, or a
    short diagnostic string when the command is rejected (twist is zero).
    """
    zero = Twist()
    speed = drive.drive.speed
    angle = drive.drive.steering_angle
    stamp = drive.header.stamp.sec + drive.header.stamp.nanosec * 1e-9

    if not (math.isfinite(speed) and math.isfinite(angle)
            and math.isfinite(stamp)):
        return zero, 'non-finite value in drive command'

    age = now_sec - stamp
    if age > MAX_STAMP_AGE:
        return zero, f'stale command: stamp age {age:.3f} s > {MAX_STAMP_AGE} s'

    v = min(max(speed, 0.0), MAX_SPEED)
    d = min(max(angle, -MAX_STEER_ANGLE), MAX_STEER_ANGLE)
    # Gazebo's AckermannSteering takes body velocities and computes the
    # wheel angles itself; a central steering angle becomes a yaw rate.
    # At v = 0 the yaw rate is 0: an Ackermann chassis cannot spin in place.
    twist = Twist()
    twist.linear.x = v
    twist.angular.z = v * math.tan(d) / WHEEL_BASE
    return twist, None


class DriveAdapter(Node):
    def __init__(self):
        super().__init__('drive_adapter')

        # Commands are a reliable stream (contract), sensors are not.
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.sub = self.create_subscription(
            AckermannDriveStamped, 'drive/safe', self._on_drive, cmd_qos)
        self.pub = self.create_publisher(Twist, 'vehicle/gz_cmd_vel', cmd_qos)

    def _on_drive(self, msg: AckermannDriveStamped):
        now = self.get_clock().now().nanoseconds * 1e-9
        twist, reason = compute_twist(msg, now)
        if reason is not None:
            self.get_logger().warning(f'rejected /drive/safe: {reason}')
        self.pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = DriveAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
