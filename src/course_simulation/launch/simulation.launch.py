"""PR07: one launch file for the whole simulation.

Starts and stays alive until Ctrl+C:
  - Gazebo server (headless: -s -r --headless-rendering, sensors need ogre2)
  - robot_state_publisher (static tf tree from PR06)
  - a single ros_gz parameter_bridge from config/bridge.yaml
  - drive adapter  (/drive/safe -> /vehicle/gz_cmd_vel)
  - odom adapter   (/vehicle/odom_gz -> /vehicle/odom + tf odom->base_link)

Contract topics (/vehicle/..., /drive/safe, /clock) stay absolute even
under a node namespace: adapters use relative names in code and are
remapped to absolute topics here. ROS nodes that check command age or
publish tf run with use_sim_time:=true.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, GroupAction, LogInfo,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from xacro import process_file

SIM_SHARE = Path(get_package_share_directory('course_simulation'))
DESC_SHARE = Path(get_package_share_directory('course_description'))


def generate_launch_description():
    world = LaunchConfiguration('world')
    bridge_config = LaunchConfiguration('bridge_config')
    headless = LaunchConfiguration('headless')
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_world = DeclareLaunchArgument(
        'world', default_value=str(SIM_SHARE / 'worlds' / 'mcity-minimal.sdf'))
    declare_bridge = DeclareLaunchArgument(
        'bridge_config', default_value=str(SIM_SHARE / 'config' / 'bridge.yaml'))
    declare_headless = DeclareLaunchArgument(
        'headless', default_value='true',
        description='Run the gz server without a GUI client; adds '
                    '--headless-rendering for camera/gpu_lidar.')
    declare_ns = DeclareLaunchArgument(
        'namespace', default_value='sim',
        description='Namespace for ROS nodes (contract topics stay absolute).')
    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true')

    # Headless: server only (-s) + run (-r) + headless-rendering for
    # camera and gpu_lidar (OGRE2/EGL). Otherwise the GUI client opens too.
    gazebo_headless = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r', '--headless-rendering', world],
        output='screen', condition=IfCondition(headless))
    gazebo_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world],
        output='screen', condition=UnlessCondition(headless))

    # robot_state_publisher: static frames from PR06, in sim time.
    robot_xml = process_file(
        str(DESC_SHARE / 'urdf' / 'vehicle.urdf.xacro')).toxml()
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_xml,
            'use_sim_time': use_sim_time,
        }],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['--ros-args', '--params-file', bridge_config],
    )

    adapters = GroupAction([
        PushRosNamespace(namespace),
        LogInfo(msg=['[course_simulation] node namespace=', namespace]),
        Node(
            package='course_simulation',
            executable='drive_adapter',
            name='drive_adapter',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[
                ('drive/safe', '/drive/safe'),
                ('vehicle/gz_cmd_vel', '/vehicle/gz_cmd_vel'),
            ],
        ),
        Node(
            package='course_simulation',
            executable='odom_adapter',
            name='odom_adapter',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[
                ('vehicle/odom_gz', '/vehicle/odom_gz'),
                ('vehicle/odom', '/vehicle/odom'),
            ],
        ),
    ])

    return LaunchDescription([
        declare_world, declare_bridge, declare_headless,
        declare_ns, declare_sim_time,
        gazebo_headless,
        gazebo_gui,
        rsp,
        bridge,
        adapters,
    ])
