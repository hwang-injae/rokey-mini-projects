"""turtlesim + gesture_robot 5개 노드 통합 실행.

5개 노드(camera/gesture/tracking/controller/main_ui) 정의는
gesture_robot 패키지의 gesture_robot.launch.py를 include해 재사용한다
(params.yaml 연동 포함, 중복 정의 방지).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    gesture_robot_launch = os.path.join(
        get_package_share_directory("gesture_robot"), "launch", "gesture_robot.launch.py"
    )

    turtlesim_node = Node(
        package="turtlesim",
        executable="turtlesim_node",
        name="turtlesim_node",
        output="screen",
    )

    # tracking/object(손 위치 또는 HSV 추적 결과)를 turtlesim 위 'target' turtle로
    # 시각화한다. turtle1은 controller_node를 통해 이 위치를 따라간다.
    target_marker_node = Node(
        package="gesture_robot",
        executable="target_marker_node",
        name="target_marker_node",
        output="screen",
    )

    # cmd_vel_topic은 기본값(turtle1/cmd_vel)을 그대로 사용해 turtlesim과 바로 연결한다.
    gesture_robot_pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gesture_robot_launch)
    )

    return LaunchDescription([
        turtlesim_node,
        target_marker_node,
        gesture_robot_pipeline,
    ])
