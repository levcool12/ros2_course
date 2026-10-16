from course_bringup.pose_relay import make_qos


def test_make_qos_reliable():
    q = make_qos('reliable')
    assert q.reliability.name == 'RELIABLE'
    assert q.depth == 10


def test_make_qos_best_effort():
    q = make_qos('best_effort')
    assert q.reliability.name == 'BEST_EFFORT'
    assert q.depth == 10


def test_make_qos_defaults_to_best_effort():
    q = make_qos('что-то-неизвестное')
    assert q.reliability.name == 'BEST_EFFORT'
