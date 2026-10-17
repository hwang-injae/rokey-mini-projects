"""Gazebo(turtlebot3_gazebo + ros_gz_bridge) 기반 통합 시뮬레이션 실행.

turtlesim 대신 Gazebo에서 turtlebot3(burger)를 스폰하고, controller_node가
발행하는 geometry_msgs/msg/Twist를 ros_gz_bridge로 Gazebo에 전달해 제스처
기반 이동을 확인한다. Gazebo GUI는 Tkinter UI와 분리된 별도 창으로 띄운다.

주의: 이 환경(ROS2 Jazzy)에는 구버전 `gazebo_ros`가 아니라 `ros_gz_sim` /
`ros_gz_bridge` 및 `turtlebot3_gazebo`가 설치되어 있어, 해당 패키지의 자원
(월드, 로봇 모델, robot_state_publisher launch)을 재사용한다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    tb3_gazebo_share = get_package_share_directory("turtlebot3_gazebo")
    sim_bringup_share = get_package_share_directory("sim_bringup")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    turtlebot3_model = LaunchConfiguration("turtlebot3_model", default="burger")
    x_pose = LaunchConfiguration("x_pose", default="0.0")
    y_pose = LaunchConfiguration("y_pose", default="0.0")
    # 손 추적 데모 등에서는 controller_node가 포함된 표준 5노드 파이프라인
    # 대신 별도 노드 조합(gesture_node + gazebo_pursuit_node)을 직접
    # 띄우고 싶을 때가 있어, 이 launch에서 파이프라인 포함 여부를 끌 수 있게 한다.
    launch_pipeline = LaunchConfiguration("launch_pipeline", default="true")

    world = os.path.join(tb3_gazebo_share, "worlds", "empty_world.world")
    # 화면에서 잘 보이도록 시각적 크기만 2배로 키운 커스텀 모델을 사용한다
    # (충돌/관성 등 물리 속성은 원본과 동일 - 순수 시각적 확대).
    model_sdf = os.path.join(sim_bringup_share, "models", "turtlebot3_burger_big.sdf")
    bridge_params = os.path.join(
        sim_bringup_share, "params", "turtlebot3_burger_twist_bridge.yaml"
    )
    # SDF <gui><camera> pose는 이 gz-sim 버전에서 무시되어("can't be converted
    # yet" 경고) 별도 GUI 플러그인 설정 파일로 탑뷰 카메라를 지정한다.
    gui_config = os.path.join(sim_bringup_share, "config", "topdown_gui.config")

    # Gazebo는 TURTLEBOT3_MODEL 환경변수로 로봇 모델(urdf/모델 폴더명)을 찾는다.
    set_turtlebot3_model = SetEnvironmentVariable("TURTLEBOT3_MODEL", turtlebot3_model)

    set_env_vars_resources = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        os.path.join(tb3_gazebo_share, "models"),
    )

    # 서버(물리 연산)와 클라이언트(GUI 창)를 분리 실행 -> Gazebo는 별도 창으로 표시된다.
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": ["-r -s -v2 ", world], "on_exit_shutdown": "true"}.items(),
    )
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": ["-g -v2 --gui-config ", gui_config]
        }.items(),
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_share, "launch", "robot_state_publisher.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    spawn_turtlebot_cmd = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", turtlebot3_model, "-file", model_sdf, "-x", x_pose, "-y", y_pose, "-z", "0.01"],
        output="screen",
    )

    # controller_node(Twist)와 Gazebo DiffDrive(gz.msgs.Twist)를 연결하는 브릿지.
    # turtlebot3_gazebo 기본 브릿지는 TwistStamped라 controller_node와 타입이 맞지 않아
    # cmd_vel만 Twist로 바꾼 sim_bringup 자체 설정을 사용한다.
    bridge_cmd = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["--ros-args", "-p", f"config_file:={bridge_params}"],
        output="screen",
    )

    # --- gesture_robot 5개 노드 파이프라인 ---
    # gesture_robot.launch.py를 include해 재사용한다 (params.yaml 연동 포함,
    # turtlesim_bringup.launch.py와의 노드 정의 중복 방지).
    # cmd_vel_topic을 "cmd_vel"로 override해 Gazebo 브릿지가 구독하는 토픽에 맞춘다.
    gesture_robot_launch = os.path.join(
        get_package_share_directory("gesture_robot"), "launch", "gesture_robot.launch.py"
    )
    gesture_robot_pipeline = GroupAction(
        condition=IfCondition(launch_pipeline),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gesture_robot_launch),
                launch_arguments={
                    "cmd_vel_topic": "cmd_vel",
                    "use_sim_time": use_sim_time,
                }.items(),
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("turtlebot3_model", default_value="burger"),
        DeclareLaunchArgument("x_pose", default_value="0.0"),
        DeclareLaunchArgument("y_pose", default_value="0.0"),
        DeclareLaunchArgument("launch_pipeline", default_value="true"),
        set_turtlebot3_model,
        set_env_vars_resources,
        gzserver_cmd,
        gzclient_cmd,
        robot_state_publisher_cmd,
        spawn_turtlebot_cmd,
        bridge_cmd,
        gesture_robot_pipeline,
    ])
