"""PR08: one-command localization bringup for the PR07 recording.

Starts map_server + amcl + lifecycle_manager_localization and keeps them
alive. Node names match the YAML keys exactly: the lifecycle manager
transitions map_server and amcl from unconfigured -> inactive -> active.

All nodes run with use_sim_time=true: during replay the only /clock
source is the recording itself. The absolute map path from the `map`
argument becomes map_server's yaml_filename.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')

    declare_params = DeclareLaunchArgument(
        'params_file',
        description='Absolute path to the AMCL/profile YAML '
                    '(keys: map_server, amcl, lifecycle_manager_localization).')
    declare_map = DeclareLaunchArgument(
        'map',
        description='Absolute path to the map yaml (passed to map_server '
                    'as yaml_filename).')

    log = LogInfo(msg=['[localization_case] params=', params_file,
                       ' map=', map_yaml])

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': True, 'yaml_filename': map_yaml},
        ],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file, {'use_sim_time': True}],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[params_file, {'use_sim_time': True}],
    )

    return LaunchDescription([
        declare_params, declare_map, log,
        map_server, amcl, lifecycle_manager,
    ])
