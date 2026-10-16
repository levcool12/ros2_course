from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, GroupAction, LogInfo,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    record = LaunchConfiguration('record')
    bag_dir = LaunchConfiguration('bag_dir')
    subscriber_reliability = LaunchConfiguration('subscriber_reliability')

    declare_ns = DeclareLaunchArgument(
        'namespace', default_value='robot_a')
    declare_rec = DeclareLaunchArgument(
        'record', default_value='false')
    declare_bag = DeclareLaunchArgument(
        'bag_dir', default_value='evidence/pr05/bag')
    declare_sub = DeclareLaunchArgument(
        'subscriber_reliability', default_value='best_effort')

    turtlesim = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim',
        output='screen',
    )

    system = GroupAction([
        PushRosNamespace(namespace),
        LogInfo(msg=['[course_bringup] namespace=', namespace]),
        LogInfo(msg=['[course_bringup] subscriber_reliability=',
                     subscriber_reliability]),
        Node(
            package='course_bringup',
            executable='pose_relay',
            name='pose_relay',
            parameters=[{'subscriber_reliability': subscriber_reliability}],
            output='screen',
        ),
        Node(
            package='patrol',
            executable='patrol',
            name='patrol',
            remappings=[('cmd_vel', '/turtle1/cmd_vel')],
            output='screen',
        ),
    ])

    recorder = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-s', 'mcap', '-o', bag_dir,
             '/robot_a/sensor_pose'],
        output='screen',
        condition=IfCondition(record),
    )

    return LaunchDescription([
        declare_ns, declare_rec, declare_bag, declare_sub,
        turtlesim,
        system,
        recorder,
    ])
