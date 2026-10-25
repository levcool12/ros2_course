"""Unit tests for the PR07 odometry adapter.

Frame normalization, preservation of stamp/pose/twist/covariances,
rejection of foreign frames and non-finite values.
"""

import pytest

from course_simulation.odom_adapter import normalize_odom, odom_to_tf
from nav_msgs.msg import Odometry


def make_odom(header_frame='course_ackermann/odom',
              child_frame='course_ackermann/base_link'):
    msg = Odometry()
    msg.header.stamp.sec = 42
    msg.header.stamp.nanosec = 500000000
    msg.header.frame_id = header_frame
    msg.child_frame_id = child_frame
    msg.pose.pose.position.x = 1.5
    msg.pose.pose.position.y = -0.25
    msg.pose.pose.position.z = 0.0
    msg.pose.pose.orientation.w = 1.0
    msg.twist.twist.linear.x = 0.8
    msg.twist.twist.angular.z = 0.1
    msg.pose.covariance = [0.01] * 36
    msg.twist.covariance = [0.02] * 36
    return msg


def test_frames_are_normalized():
    out = normalize_odom(make_odom())
    assert out is not None
    assert out.header.frame_id == 'odom'
    assert out.child_frame_id == 'base_link'


def test_stamp_pose_twist_covariances_preserved():
    src = make_odom()
    out = normalize_odom(src)
    assert out.header.stamp.sec == 42
    assert out.header.stamp.nanosec == 500000000
    assert out.pose.pose.position.x == pytest.approx(1.5)
    assert out.pose.pose.position.y == pytest.approx(-0.25)
    assert out.twist.twist.linear.x == pytest.approx(0.8)
    assert out.twist.twist.angular.z == pytest.approx(0.1)
    assert list(out.pose.covariance) == [0.01] * 36
    assert list(out.twist.covariance) == [0.02] * 36


def test_wrong_header_frame_rejected():
    assert normalize_odom(make_odom(header_frame='odom')) is None


def test_wrong_child_frame_rejected():
    assert normalize_odom(make_odom(child_frame='base_link')) is None


def test_non_finite_values_rejected():
    src = make_odom()
    src.pose.pose.position.x = float('nan')
    assert normalize_odom(src) is None
    src2 = make_odom()
    src2.twist.twist.linear.y = float('inf')
    assert normalize_odom(src2) is None


def test_tf_built_from_normalized_message():
    out = normalize_odom(make_odom())
    tf = odom_to_tf(out)
    assert tf.header.frame_id == 'odom'
    assert tf.child_frame_id == 'base_link'
    assert tf.header.stamp.sec == 42
    assert tf.transform.translation.x == pytest.approx(1.5)
    assert tf.transform.translation.y == pytest.approx(-0.25)
    assert tf.transform.rotation.w == pytest.approx(1.0)
