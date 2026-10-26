"""PR06: bring up robot_state_publisher (and optionally RViz) for the
course Ackermann vehicle.

The robot_description is produced by expanding the Xacro file at launch
time; wheels only show up in tf once somebody publishes /joint_states.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from xacro import process_file


def generate_launch_description():
    pkg_share = Path(get_package_share_directory('course_description'))
    xacro_path = pkg_share / 'urdf' / 'vehicle.urdf.xacro'

    # Expand Xacro here (not via Command substitution) so the same code
    # works wherever the xacro python module is importable.
    robot_xml = process_file(str(xacro_path)).toxml()

    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')

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

    # Saved RViz layout: RobotModel from /robot_description, TF markers,
    # Fixed Frame = base_link (map/odom are not published in this package).
    # Without it RViz opens with an empty grid only.
    rviz_config = pkg_share / 'rviz' / 'vehicle.rviz'
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(rviz),
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', str(rviz_config)],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use the /clock topic instead of wall time.'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Open RViz2 alongside robot_state_publisher.'),
        rsp,
        rviz_node,
    ])
