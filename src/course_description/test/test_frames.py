"""Frame contract of the vehicle tree (PR06).

frames.md is the single source of truth for who owns each transform.
This test fails if any expected parent/child pair or expected static
offset changes — for example when lidar_link is accidentally attached
to camera_link instead of base_link. check_urdf stays green in that
case: a structurally valid tree can still be geometrically wrong.
"""

import os
import xml.etree.ElementTree as ET

from xacro import process_file

URDF_XACRO = os.path.join(
    os.path.dirname(__file__), '..', 'urdf', 'vehicle.urdf.xacro')

# joint name -> (parent, child, type, expected origin xyz)
EXPECTED_FRAMES = {
    'camera_joint': ('base_link', 'camera_link', 'fixed', (0.40, 0.0, 0.105)),
    'lidar_joint': ('base_link', 'lidar_link', 'fixed', (0.0, 0.0, 0.115)),
    'imu_joint': ('base_link', 'imu_link', 'fixed', (0.10, 0.0, 0.02)),
    'front_left_wheel_joint':
        ('base_link', 'front_left_wheel_link', 'continuous', (0.4, 0.3, 0.0)),
    'front_right_wheel_joint':
        ('base_link', 'front_right_wheel_link', 'continuous', (0.4, -0.3, 0.0)),
    'rear_left_wheel_joint':
        ('base_link', 'rear_left_wheel_link', 'continuous', (-0.4, 0.3, 0.0)),
    'rear_right_wheel_joint':
        ('base_link', 'rear_right_wheel_link', 'continuous', (-0.4, -0.3, 0.0)),
}


def expanded():
    doc = process_file(URDF_XACRO)
    return ET.fromstring(doc.toxml())


def test_expected_parents_and_children():
    root = expanded()
    actual = {j.get('name'): j for j in root.findall('joint')}

    for name, (parent, child, _type, _xyz) in EXPECTED_FRAMES.items():
        assert name in actual, f'missing joint {name}'
        joint = actual[name]
        assert joint.find('parent').get('link') == parent, (
            f"{name}: parent is "
            f"{joint.find('parent').get('link')}, expected {parent}")
        assert joint.find('child').get('link') == child, (
            f"{name}: child is {joint.find('child').get('link')}, expected {child}")


def test_expected_transforms_and_types():
    root = expanded()
    actual = {j.get('name'): j for j in root.findall('joint')}

    for name, (_p, _c, jtype, xyz) in EXPECTED_FRAMES.items():
        joint = actual[name]
        assert joint.get('type') == jtype, f'{name}: type {joint.get("type")}'
        origin = joint.find('origin')
        actual_xyz = tuple(
            round(float(v), 6) for v in (origin.get('xyz') or '0 0 0').split())
        assert actual_xyz == tuple(round(v, 6) for v in xyz), (
            f'{name}: origin {actual_xyz}, expected {xyz}')
