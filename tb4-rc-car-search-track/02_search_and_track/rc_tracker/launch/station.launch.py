"""Webcam PC (webcam + control station): webcam_detector + mission_trigger.

The USB webcam (DRGO-WC720) is plugged into this machine.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('rc_tracker')
    webcam_cfg = os.path.join(pkg, 'config', 'webcam_params.yaml')
    mission_cfg = os.path.join(pkg, 'config', 'mission_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('webcam_params', default_value=webcam_cfg),
        DeclareLaunchArgument('mission_params', default_value=mission_cfg),
        Node(
            package='rc_tracker',
            executable='webcam_detector',
            name='webcam_detector_node',
            output='screen',
            parameters=[LaunchConfiguration('webcam_params')],
        ),
        Node(
            package='rc_tracker',
            executable='mission_trigger',
            name='mission_trigger_node',
            output='screen',
            parameters=[LaunchConfiguration('mission_params')],
        ),
    ])
