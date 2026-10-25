"""Bridge configuration checks for PR07.

The bridge YAML is part of the contract: a wrong direction or a wrong
topic name is invisible in a running system until data silently stops.
These tests validate the real config against contracts/topics.md and
prove the validator catches the defect class of the stage-2 broken
copy (config/bridge-broken.yaml).
"""

import os

import pytest
import yaml

from ament_index_python.packages import get_package_share_directory

CONFIG_DIR = os.path.join(
    os.path.dirname(__file__), '..', 'config')

# ros_topic -> (direction, gz_topic, ros_type)
CONTRACT = {
    '/vehicle/gz_cmd_vel': (
        'ROS_TO_GZ', '/model/course_ackermann/cmd_vel',
        'geometry_msgs/msg/Twist'),
    '/vehicle/camera/image_raw': (
        'GZ_TO_ROS', '/model/course_ackermann/camera/image',
        'sensor_msgs/msg/Image'),
    '/vehicle/scan': (
        'GZ_TO_ROS', '/model/course_ackermann/scan',
        'sensor_msgs/msg/LaserScan'),
    '/vehicle/imu': (
        'GZ_TO_ROS', '/model/course_ackermann/imu',
        'sensor_msgs/msg/Imu'),
    '/vehicle/odom_gz': (
        'GZ_TO_ROS', '/model/course_ackermann/odometry',
        'nav_msgs/msg/Odometry'),
    '/clock': (
        'GZ_TO_ROS', '/clock', 'rosgraph_msgs/msg/Clock'),
}


def load_config(name):
    path = os.path.join(CONFIG_DIR, name)
    with open(path, encoding='utf-8') as f:
        return {e['ros_topic_name']: e for e in yaml.safe_load(f)}


def problems(entries):
    """List every contract violation found in a bridge config."""
    found = []
    for ros_topic, (direction, gz_topic, ros_type) in CONTRACT.items():
        entry = entries.get(ros_topic)
        if entry is None:
            found.append(f'{ros_topic}: missing bridge entry')
            continue
        if entry.get('direction') != direction:
            found.append(
                f"{ros_topic}: direction {entry.get('direction')} != {direction}")
        if entry.get('gz_topic_name') != gz_topic:
            found.append(
                f"{ros_topic}: gz topic {entry.get('gz_topic_name')} != {gz_topic}")
        if entry.get('ros_type_name') != ros_type:
            found.append(
                f"{ros_topic}: type {entry.get('ros_type_name')} != {ros_type}")
    # One bridge only: duplicates would create competing publishers.
    if len(entries) != len(set(entries)):
        found.append('duplicate ros_topic_name entries')
    return found


def test_real_bridge_config_matches_contract():
    entries = load_config('bridge.yaml')
    assert set(entries) == set(CONTRACT), 'unexpected extra bridge entries'
    assert problems(entries) == []


def test_broken_copy_is_detected():
    # Stage 2 defect class: wrong IMU direction + misspelled cmd_vel topic.
    entries = load_config('bridge-broken.yaml')
    found = problems(entries)
    assert any('/vehicle/imu' in p and 'direction' in p for p in found)
    assert any('/vehicle/gz_cmd_vel' in p and 'gz topic' in p for p in found)


def test_no_second_bridge_instance_in_launch():
    # The single bridge must come from one config file; the launch file
    # must not spawn parameter_bridge twice.
    launch_path = os.path.join(
        os.path.dirname(__file__), '..', 'launch', 'simulation.launch.py')
    with open(launch_path, encoding='utf-8') as f:
        text = f.read()
    # Count actual bridge node instantiations, not docstring mentions.
    assert text.count("executable='parameter_bridge'") == 1, \
        'more than one bridge spawned'
    assert 'drive/raw' not in text, '/drive/raw must not be bridged'
