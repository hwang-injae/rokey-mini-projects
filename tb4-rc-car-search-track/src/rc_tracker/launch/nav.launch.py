"""Nav2 + AMCL localization with the mission's packaged map and tuned costmap.

One command replaces the two turtlebot4_navigation launches:
  ros2 launch rc_tracker nav.launch.py namespace:=/robot2

Overrides vs the turtlebot4_navigation defaults:
  - map        : rc_tracker/maps/teleop_map.yaml (default path; the map itself is
                 not in this repository -> pass map:=<your_map>.yaml)
  - nav2 params: rc_tracker/config/nav2.yaml   (inflation_radius 0.45 -> 0.25)

Still set the AMCL initial pose afterwards (RViz 2D Pose Estimate, or):
  ros2 topic pub -1 /robot2/initialpose geometry_msgs/msg/PoseWithCovarianceStamped \\
    "{header: {frame_id: 'map'}, pose: {pose: {position: {x: -0.055, y: -0.167}, \\
      orientation: {w: 1.0}}}}"
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    rc_share = get_package_share_directory('rc_tracker')
    nav_share = get_package_share_directory('turtlebot4_navigation')

    ns = LaunchConfiguration('namespace')
    map_yaml = LaunchConfiguration('map')
    nav2_params = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='/robot2'),
        DeclareLaunchArgument(
            'map', default_value=os.path.join(rc_share, 'maps', 'teleop_map.yaml')),
        DeclareLaunchArgument(
            'params_file', default_value=os.path.join(rc_share, 'config', 'nav2.yaml')),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_share, 'launch', 'localization.launch.py')),
            launch_arguments={'namespace': ns, 'map': map_yaml}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_share, 'launch', 'nav2.launch.py')),
            launch_arguments={'namespace': ns, 'params_file': nav2_params}.items(),
        ),
    ])
