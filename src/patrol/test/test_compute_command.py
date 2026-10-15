from patrol.patrol import compute_command


def test_missing_pose_returns_zero():
    cmd = compute_command(None)
    assert cmd.linear.x == 0.0
    assert cmd.angular.z == 0.0


def test_normal_pose_returns_forward_and_turn():
    class DummyPose:
        pass

    pose = DummyPose()
    cmd = compute_command(pose)
    assert cmd.linear.x == 0.5
    assert cmd.angular.z == 0.3
