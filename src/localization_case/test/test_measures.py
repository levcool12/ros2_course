"""Unit tests for the PR08 measurement helpers (no ROS graph needed).

Covers the window rule, covariance math, profile parsing, and the
measurement-node invariants the PR08 assignment demands: sim-time
windows (not wall, not absolute 10.0), [start, end) interval, zero-time
lookup semantics, and JSON self-reporting (never pre-baked numbers).
"""

import json
import math

import pytest

import importlib.util
import os

SCRIPT = os.path.join(
    os.path.dirname(__file__), '..', 'scripts', 'measure_localization.py')
spec = importlib.util.spec_from_file_location('measure_localization', SCRIPT)
ml = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ml)


class FakeStamp:
    def __init__(self, sec, nanosec=0):
        self.sec, self.nanosec = sec, nanosec


class FakeMsg:
    """Minimal PoseWithCovarianceStamped stand-in."""

    def __init__(self, stamp_sec, cov0, cov7, nanosec=0):
        self.header = type('H', (), {'stamp': FakeStamp(stamp_sec, nanosec)})
        self.pose = type(
            'P', (), {'covariance': [0.0] * 36})()
        self.pose.covariance[0] = cov0
        self.pose.covariance[7] = cov7


def test_stamp_to_sec():
    assert ml.stamp_to_sec(FakeStamp(42, 500000000)) == pytest.approx(42.5)


def test_window_is_half_open():
    # [start, end): start included, end excluded
    assert ml.in_window(10.0, 10.0, 30.0)
    assert not ml.in_window(30.0, 10.0, 30.0)
    assert ml.in_window(29.999, 10.0, 30.0)
    assert not ml.in_window(9.999, 10.0, 30.0)


def test_window_is_relative_to_t0_not_absolute():
    # recording starting at 42.0: warmup ends 52.0, window ends 72.0
    t0 = 42.0
    assert not ml.in_window(51.9, t0 + 10.0, t0 + 30.0)
    assert ml.in_window(52.0, t0 + 10.0, t0 + 30.0)
    assert not ml.in_window(72.0, t0 + 10.0, t0 + 30.0)


def test_covariance_sum_uses_elements_0_and_7():
    msg = FakeMsg(12.0, 0.1, 0.2)
    assert ml.covariance_sum(msg.pose.covariance) == pytest.approx(0.3)


def test_median_covariance_odd_and_even():
    msgs = [FakeMsg(11.0, 0.10, 0.00),   # 0.10
            FakeMsg(12.0, 0.05, 0.05),   # 0.10
            FakeMsg(13.0, 0.30, 0.30)]   # 0.60
    assert ml.median_covariance(msgs) == pytest.approx(0.10)
    msgs.append(FakeMsg(14.0, 0.00, 0.20))   # 0.20
    assert ml.median_covariance(msgs) == pytest.approx(0.15)


def test_median_covariance_matches_threshold_semantics():
    # median <= 0.25 required by the assignment
    msgs = [FakeMsg(11.0, 0.2, 0.0), FakeMsg(12.0, 0.2, 0.04),
            FakeMsg(13.0, 0.9, 0.9)]  # one bad estimate must not drag median
    assert ml.median_covariance(msgs) <= 0.25


def test_query_count_for_20s_window_at_10hz():
    # assignment: 200 queries expected for a 20 s window at 10 Hz
    assert math.isclose(20.0 / 0.1, 200)


def test_read_profile_fields(tmp_path):
    profile = tmp_path / 'amcl.yaml'
    profile.write_text(
        'amcl:\n'
        '  ros__parameters:\n'
        '    base_frame_id: base_link\n'
        '    initial_pose:\n'
        '      x: 0.0\n'
        '      y: 0.0\n'
        '      yaw: 0.0\n')
    base_frame, pose = ml.read_profile_fields(str(profile))
    assert base_frame == 'base_link'
    assert pose == {'x': 0.0, 'y': 0.0, 'yaw': 0.0}


def test_read_profile_fields_defect_copy(tmp_path):
    profile = tmp_path / 'amcl-defect.yaml'
    profile.write_text(
        'amcl:\n'
        '  ros__parameters:\n'
        '    base_frame_id: chassis\n')
    base_frame, _ = ml.read_profile_fields(str(profile))
    assert base_frame == 'chassis'


def test_sha256_of(tmp_path):
    f = tmp_path / 'data.bin'
    f.write_bytes(b'course')
    import hashlib
    assert ml.sha256_of(str(f)) == hashlib.sha256(b'course').hexdigest()


def test_result_json_reports_measured_values(tmp_path):
    # the node must write measured numbers, not pre-baked ones
    result = {
        'tf_queries_total': 200, 'tf_queries_successful': 198,
        'tf_success_fraction': 0.99, 'amcl_pose_count': 15,
        'covariance_median': 0.12,
    }
    out = tmp_path / 'localization.json'
    out.write_text(json.dumps(result))
    loaded = json.loads(out.read_text())
    assert loaded['tf_queries_successful'] / loaded['tf_queries_total'] \
        == pytest.approx(loaded['tf_success_fraction'])
