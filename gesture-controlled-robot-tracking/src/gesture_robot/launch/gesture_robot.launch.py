"""전체 시스템 통합 실행 구성을 관리하는 Launch 파일.

turtlesim/Gazebo 등 시뮬레이터별 실행은 sim_bringup 패키지의 각 bringup
launch가 이 파일을 include하여 구성한다 (5개 노드 정의 중복 방지).
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """팀에서 구현한 노드들과 파라미터(params.yaml)를 등록한다."""
    pkg_dir = get_package_share_directory("gesture_robot")
    param_file = os.path.join(pkg_dir, "config", "params.yaml")

    # turtlesim은 기본값(turtle1/cmd_vel) 그대로 사용하고,
    # Gazebo 등 다른 대상은 이 인자로 실제 구독 토픽에 맞춰 remap한다.
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic", default="turtle1/cmd_vel")
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    camera_node = Node(
        package="gesture_robot",
        executable="camera_node",
        name="camera_node",
        parameters=[param_file, {"use_sim_time": use_sim_time}],
        output="screen",
    )

    gesture_node = Node(
        package="gesture_robot",
        executable="gesture_node",
        name="gesture_node",
        parameters=[param_file, {"use_sim_time": use_sim_time}],
        output="screen",
    )

    object_tracking_node = Node(
        package="gesture_robot",
        executable="object_tracking_node",
        name="object_tracking_node",
        parameters=[param_file, {"use_sim_time": use_sim_time}],
        output="screen",
    )

    controller_node = Node(
        package="gesture_robot",
        executable="controller_node",
        name="controller_node",
        parameters=[param_file, {"use_sim_time": use_sim_time}],
        output="screen",
        remappings=[("turtle1/cmd_vel", cmd_vel_topic)],
    )

    main_ui_node = Node(
        package="gesture_robot",
        executable="main_ui",
        name="main_ui",
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "cmd_vel_topic",
                default_value="turtle1/cmd_vel",
                description="controller_node의 속도 명령을 remap할 대상 토픽",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Gazebo 등 시뮬레이션 클럭 사용 여부",
            ),
            camera_node,
            gesture_node,
            object_tracking_node,
            controller_node,
            main_ui_node,
        ]
    )
