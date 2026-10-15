from patrol.patrol import compute_command, validate_param


def test_missing_pose_returns_zero():
    cmd = compute_command(None, 0.5, 0.3)
    assert cmd.linear.x == 0.0
    assert cmd.angular.z == 0.0


def test_normal_pose_returns_forward_and_turn():
    class DummyPose:
        pass

    cmd = compute_command(DummyPose(), 0.5, 0.3)
    assert cmd.linear.x == 0.5
    assert cmd.angular.z == 0.3


def test_publish_hz_normal_change_10_to_5_is_valid():
    ok, _ = validate_param('publish_hz', 5.0)
    assert ok


def test_publish_hz_zero_is_rejected():
    ok, reason = validate_param('publish_hz', 0.0)
    assert not ok
    assert 'publish_hz' in reason


def test_publish_hz_negative_is_rejected():
    ok, _ = validate_param('publish_hz', -1.0)
    assert not ok


def test_publish_hz_nan_is_rejected():
    ok, _ = validate_param('publish_hz', float('nan'))
    assert not ok


def test_publish_hz_above_30_is_rejected():
    ok, _ = validate_param('publish_hz', 100.0)
    assert not ok


def test_linear_speed_out_of_range():
    assert not validate_param('linear_speed', -0.1)[0]
    assert not validate_param('linear_speed', 1.5)[0]


def test_turn_rate_out_of_range():
    assert not validate_param('turn_rate', -1.5)[0]
    assert not validate_param('turn_rate', 2.0)[0]


def test_nan_speed_is_rejected():
    assert not validate_param('linear_speed', float('nan'))[0]
