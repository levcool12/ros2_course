"""PR07 odometry adapter: normalizes Gazebo odometry and owns the
single dynamic tf odom -> base_link.

The model's AckermannSteering plugin publishes nav_msgs/Odometry with
model-scoped frames:  header.frame_id = "course_ackermann/odom",
child_frame_id = "course_ackermann/base_link".  This adapter renames
them to the conventional odom / base_link, keeps stamp, pose, twist and
both covariance matrices, and republishes on /vehicle/odom.  Messages
with other source frames or non-finite values are rejected
(contracts/topics.md).  It also publishes exactly one dynamic transform
odom -> base_link per message; the static links of PR06 stay with
robot_state_publisher, so every tf edge has a single owner.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from tf2_ros import TransformBroadcaster

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

SRC_FRAME = 'course_ackermann/odom'
SRC_CHILD = 'course_ackermann/base_link'
DST_FRAME = 'odom'
DST_CHILD = 'base_link'


def normalize_odom(msg: Odometry):
    """Return a normalized copy of msg, or None when it is rejected."""
    if msg.header.frame_id != SRC_FRAME or msg.child_frame_id != SRC_CHILD:
        return None
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    t = msg.twist.twist
    if not all(math.isfinite(v) for v in (
            p.x, p.y, p.z, q.x, q.y, q.z, q.w,
            t.linear.x, t.linear.y, t.linear.z,
            t.angular.x, t.angular.y, t.angular.z)):
        return None

    out = Odometry()
    out.header = msg.header                      # stamp preserved
    out.header.frame_id = DST_FRAME
    out.child_frame_id = DST_CHILD
    out.pose = msg.pose                          # pose + covariance preserved
    out.twist = msg.twist                        # twist + covariance preserved
    return out


def odom_to_tf(msg: Odometry) -> TransformStamped:
    """Build the odom -> base_link transform from a normalized message."""
    tf = TransformStamped()
    tf.header = msg.header
    tf.header.frame_id = DST_FRAME
    tf.child_frame_id = DST_CHILD
    tf.transform.translation.x = msg.pose.pose.position.x
    tf.transform.translation.y = msg.pose.pose.position.y
    tf.transform.translation.z = msg.pose.pose.position.z
    tf.transform.rotation = msg.pose.pose.orientation
    return tf


class OdomAdapter(Node):
    def __init__(self):
        super().__init__('odom_adapter')

        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.sub = self.create_subscription(
            Odometry, 'vehicle/odom_gz', self._on_odom, odom_qos)
        self.pub = self.create_publisher(Odometry, 'vehicle/odom', odom_qos)
        self.tf_pub = TransformBroadcaster(self)

    def _on_odom(self, msg: Odometry):
        out = normalize_odom(msg)
        if out is None:
            self.get_logger().warning(
                f'rejected odometry: frames {msg.header.frame_id} / '
                f'{msg.child_frame_id} or non-finite values')
            return
        self.pub.publish(out)
        self.tf_pub.sendTransform(odom_to_tf(out))


def main(args=None):
    rclpy.init(args=args)
    node = OdomAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
