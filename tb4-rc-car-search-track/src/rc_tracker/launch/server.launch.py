"""GPU laptop: search_and_track_server only.

Nav2 + localization (AMCL + map_server) run separately, started first with:
  ros2 launch rc_tracker nav.launch.py namespace:=/robot2 map:=<your_map>.yaml
See docs/run.md for the full order (undock -> nav -> server -> station).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_cfg = os.path.join(
        get_package_share_directory('rc_tracker'), 'config', 'mission_params.yaml')
    ns = LaunchConfiguration('namespace')
    params = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='/robot2'),
        DeclareLaunchArgument('params_file', default_value=default_cfg),
        Node(
            package='rc_tracker',
            executable='search_and_track_server',
            name='search_and_track_server',
            output='screen',
            parameters=[params, {'robot_namespace': ns}],
            # TF is published under the robot namespace (/robot2/tf), but a plain
            # TransformListener subscribes to /tf. Redirect it.
            remappings=[
                ('/tf', [ns, '/tf']),
                ('/tf_static', [ns, '/tf_static']),
            ],
        ),
    ])
