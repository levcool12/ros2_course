"""Structural checks of the expanded vehicle URDF (PR06).

These tests prove the model is a valid, physically plausible tree in SI
units. They intentionally go beyond check_urdf: structural correctness
does not prove geometry.
"""

import os
import xml.etree.ElementTree as ET

from xacro import process_file

URDF_XACRO = os.path.join(
    os.path.dirname(__file__), '..', 'urdf', 'vehicle.urdf.xacro')

# Default parameter values, mirrored from the xacro args.
DEFAULTS = {
    'body_length': 0.9,
    'body_width': 0.5,
    'body_height': 0.15,
    'wheelbase': 0.8,
    'track': 0.6,
    'wheel_radius': 0.12,
}


def expand(**mappings):
    doc = process_file(URDF_XACRO, mappings=mappings or None)
    return ET.fromstring(doc.toxml())


def joints(root):
    return {j.get('name'): j for j in root.findall('joint')}


def links(root):
    return {l.get('name'): l for l in root.findall('link')}


def origin_xyz(joint):
    origin = joint.find('origin')
    if origin is None or origin.get('xyz') is None:
        return (0.0, 0.0, 0.0)
    return tuple(float(v) for v in origin.get('xyz').split())


def inertia_of(link):
    return link.find('inertial/inertia')


def test_tree_is_connected_and_acyclic():
    root = expand()
    all_links = set(links(root))
    parent_of = {
        j.find('child').get('link'): j.find('parent').get('link')
        for j in root.findall('joint')}

    # Every joint child has exactly one parent and no link parents itself.
    assert len(parent_of) == len(joints(root)), 'a link has two parents'
    for child, parent in parent_of.items():
        assert child in all_links and parent in all_links
        assert child != parent

    # Walking up from every link reaches the root in <= n steps (acyclic).
    for link in all_links:
        seen, cur = set(), link
        while cur != 'base_link':
            assert cur in parent_of, f'{link} is not connected to base_link'
            assert cur not in seen, f'cycle detected via {cur}'
            seen.add(cur)
            cur = parent_of[cur]

    assert len(all_links) == len(joints(root)) + 1


def test_joint_types():
    jnts = joints(expand())
    assert jnts['camera_joint'].get('type') == 'fixed'
    assert jnts['lidar_joint'].get('type') == 'fixed'
    assert jnts['imu_joint'].get('type') == 'fixed'
    for name in ('front_left_wheel_joint', 'front_right_wheel_joint',
                 'rear_left_wheel_joint', 'rear_right_wheel_joint'):
        assert jnts[name].get('type') == 'continuous', name


def test_geometry_is_parameterized_no_magic_numbers():
    # Defaults put the axles at +-wheelbase/2 and +-track/2, with the
    # wheel radius taken from the parameter.
    jnts = joints(expand())
    assert origin_xyz(jnts['front_left_wheel_joint']) == \
        (DEFAULTS['wheelbase'] / 2, DEFAULTS['track'] / 2, 0.0)
    assert origin_xyz(jnts['rear_right_wheel_joint']) == \
        (-DEFAULTS['wheelbase'] / 2, -DEFAULTS['track'] / 2, 0.0)

    # Re-expanding with different arguments actually moves the joints:
    # the numbers come from the parameters, not from hard-coded values.
    jnts = joints(expand(wheelbase='1.0', track='0.8', wheel_radius='0.15'))
    assert origin_xyz(jnts['front_left_wheel_joint']) == (0.5, 0.4, 0.0)
    wheel = links(expand(wheel_radius='0.15'))['front_left_wheel_link']
    cyl = wheel.find('visual/geometry/cylinder')
    assert float(cyl.get('radius')) == 0.15


def test_visual_and_collision_are_separate():
    root = expand()
    for name, link in links(root).items():
        assert link.find('visual/geometry') is not None, f'{name}: no visual'
        assert link.find('collision/geometry') is not None, f'{name}: no collision'


def test_inertia_is_physically_plausible():
    root = expand()
    for name, link in links(root).items():
        inertial = link.find('inertial')
        assert inertial is not None, f'{name}: no inertial'
        mass = float(inertial.find('mass').get('value'))
        assert mass > 0.0, f'{name}: mass must be positive'

        inertia = inertial.find('inertia')
        ixx = float(inertia.get('ixx'))
        iyy = float(inertia.get('iyy'))
        izz = float(inertia.get('izz'))
        assert min(ixx, iyy, izz) > 0.0, f'{name}: inertia not positive'
        # Triangle inequalities of a positive-definite diagonal tensor.
        assert ixx + iyy >= izz, name
        assert ixx + izz >= iyy, name
        assert iyy + izz >= ixx, name


def test_units_are_metric_scale():
    # Every origin and geometry stays within a plausible robot scale
    # (guards against a unit slip like cm instead of m).
    root = expand()
    for joint in root.findall('joint'):
        for value in origin_xyz(joint):
            assert abs(value) < 5.0, joint.get('name')
    for box in root.iter('box'):
        for value in (float(v) for v in box.get('size').split()):
            assert 0.0 < value < 5.0
