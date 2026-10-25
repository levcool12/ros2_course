"""Unit tests for the PR07 drive adapter conversion.

Covers the required cases: straight driving, turning, zero speed,
stale command, and non-finite command - plus clamping.
"""

import math

import pytest

from ackermann_msgs.msg import AckermannDriveStamped
from course_simulation.drive_adapter import (
    compute_twist, MAX_SPEED, MAX_STEER_ANGLE, WHEEL_BASE,
)

NOW = 100.0


def make_drive(speed, angle, stamp_sec=NOW):
    msg = AckermannDriveStamped()
    msg.header.stamp.sec = int(stamp_sec)
    msg.header.stamp.nanosec = int(round((stamp_sec % 1) * 1e9))
    msg.drive.speed = speed
    msg.drive.steering_angle = angle
    return msg


def test_straight_motion():
    twist, reason = compute_twist(make_drive(0.5, 0.0), NOW)
    assert reason is None
    assert twist.linear.x == pytest.approx(0.5)
    assert twist.angular.z == pytest.approx(0.0)


def test_turning_motion():
    twist, reason = compute_twist(make_drive(0.5, 0.3), NOW)
    assert reason is None
    assert twist.linear.x == pytest.approx(0.5)
    # omega = v * tan(delta) / L  with L = 0.8
    assert twist.angular.z == pytest.approx(0.5 * math.tan(0.3) / WHEEL_BASE)


def test_zero_speed_means_zero_twist():
    # Ackermann chassis cannot turn in place: v = 0 -> omega = 0.
    twist, reason = compute_twist(make_drive(0.0, 0.3), NOW)
    assert reason is None
    assert twist.linear.x == pytest.approx(0.0)
    assert twist.angular.z == pytest.approx(0.0)


def test_values_are_clamped():
    twist, reason = compute_twist(make_drive(5.0, 10.0), NOW)
    assert reason is None
    assert twist.linear.x == pytest.approx(MAX_SPEED)
    assert twist.angular.z == pytest.approx(
        MAX_SPEED * math.tan(MAX_STEER_ANGLE) / WHEEL_BASE)


def test_negative_speed_clamps_to_zero():
    twist, reason = compute_twist(make_drive(-0.5, 0.0), NOW)
    assert reason is None
    assert twist.linear.x == pytest.approx(0.0)


def test_stale_command_rejected():
    twist, reason = compute_twist(make_drive(0.5, 0.0, stamp_sec=NOW - 0.4), NOW)
    assert reason is not None and 'stale' in reason
    assert twist.linear.x == 0.0 and twist.angular.z == 0.0


def test_fresh_command_accepted_at_boundary():
    twist, reason = compute_twist(make_drive(0.5, 0.0, stamp_sec=NOW - 0.2), NOW)
    assert reason is None
    assert twist.linear.x == pytest.approx(0.5)


def test_unstamped_command_is_stale():
    # stamp 0.0 while sim time is far ahead -> rejected as stale.
    twist, reason = compute_twist(make_drive(0.5, 0.0, stamp_sec=0.0), NOW)
    assert reason is not None and 'stale' in reason
    assert twist.linear.x == 0.0


def test_nan_speed_rejected():
    twist, reason = compute_twist(make_drive(float('nan'), 0.0), NOW)
    assert reason is not None and 'non-finite' in reason
    assert twist.linear.x == 0.0 and twist.angular.z == 0.0


def test_nan_steering_rejected():
    twist, reason = compute_twist(make_drive(0.5, float('nan')), NOW)
    assert reason is not None and 'non-finite' in reason
    assert twist.linear.z == 0.0 and twist.angular.z == 0.0


def test_inf_speed_rejected():
    twist, reason = compute_twist(make_drive(float('inf'), 0.0), NOW)
    assert reason is not None and 'non-finite' in reason
    assert twist.linear.x == 0.0
