"""추적 상태와 로봇 이동 제어 ROS2 노드."""

from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
from geometry_msgs.msg import Twist
from gesture_robot_interfaces.msg import GestureCommand, TrackedObject
from gesture_robot.core.tracking_controller import TrackingController, VelocityCommand
from gesture_robot.core.tracking_state import TrackingStateMachine


class ControllerNode(Node):
    """제스처와 객체 추적 결과를 이동 명령으로 변환한다."""

    def __init__(self) -> None:
        super().__init__("controller_node")

        self._state_machine = TrackingStateMachine()
        self._declare_parameters()
        self._controller = self._create_controller()
        self._tracking_timeout_sec = float(
            self.get_parameter("tracking_timeout_sec").value
        )
        self._last_tracking_time: Optional[Time] = None

        self._setup_qos()
        self._setup_subscribers()
        self._setup_publisher()
        self._timeout_timer = self.create_timer(0.1, self._on_timeout_timer)

        self.get_logger().info("controller_node 시작됨")

    def _declare_parameters(self) -> None:
        """이동 제어 파라미터를 선언한다."""
        # TODO: 이동 제어 기본값은 실제 카메라와 turtlesim 통합 테스트 후
        # 팀 합의값으로 조정한다.
        self.declare_parameters(
            namespace="",
            parameters=[
                ("linear_gain", 0.004),
                ("angular_gain", 0.004),
                ("target_area", 2000.0),
                ("area_deadband", 100.0),
                ("angular_deadband", 10.0),
                ("max_linear_speed", 2.0),
                ("max_angular_speed", 2.0),
                ("tracking_timeout_sec", 0.5),
            ],
        )

    def _create_controller(self) -> TrackingController:
        """ROS2 파라미터로 제어 알고리즘을 생성한다."""
        return TrackingController(
            linear_gain=float(self.get_parameter("linear_gain").value),
            angular_gain=float(self.get_parameter("angular_gain").value),
            target_area=float(self.get_parameter("target_area").value),
            area_deadband=float(self.get_parameter("area_deadband").value),
            angular_deadband=float(self.get_parameter("angular_deadband").value),
            max_linear_speed=float(self.get_parameter("max_linear_speed").value),
            max_angular_speed=float(self.get_parameter("max_angular_speed").value),
        )

    # 통신 품질 설정
    def _setup_qos(self) -> None:
        """QoS 프로필을 설정한다."""
        # RELIABLE: 데이터가 중간에 유실되지 않고 반드시 도착하도록 보장
        self._reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        # BEST_EFFORT: 데이터 유실을 감수하더라도 가장 최신 데이터를 빨리 보냄
        self._best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

    # 제스쳐 인식하는 구독자와 공 객체 탐지 구독자 생성
    def _setup_subscribers(self) -> None:
        """Subscriber를 생성한다."""
        self._gesture_sub = self.create_subscription(
            GestureCommand,
            "gesture/command",
            self._on_gesture_command,
            self._reliable_qos,
        )
        self._tracking_sub = self.create_subscription(
            TrackedObject,
            "tracking/object",
            self._on_tracking_result,
            self._best_effort_qos,
        )

    # 거북이에게 명령 내릴 퍼블리셔
    def _setup_publisher(self) -> None:
        """Publisher를 생성한다."""
        self._velocity_pub = self.create_publisher(
            Twist,
            "turtle1/cmd_vel",
            self._reliable_qos,
        )

    # 제스쳐가 들어올때마다 상태 업데이트
    def _on_gesture_command(self, msg: GestureCommand) -> None:
        """제스처 명령을 수신할 때 호출된다."""
        self._state_machine.update(msg.command)
        state = self._state_machine.current_state()
        self.get_logger().info(
            f"제스처 수신: command={msg.command}, state={state}"
        )
        if msg.command == GestureCommand.STOP:
            self._publish_velocity(VelocityCommand())

    # 공 객체 추적 결과가 들어올때마다 상태 업데이트
    def _on_tracking_result(self, msg: TrackedObject) -> None:
        """객체 추적 결과를 수신할 때 호출된다."""
        self._last_tracking_time = self.get_clock().now()

        # 속도 계산
        vel_cmd = self._controller.calculate(
            tracking_active=self._state_machine.is_active(),
            detected=msg.detected,
            error_x=msg.error_x,
            error_y=msg.error_y,
            area=msg.area,
        )

        self._publish_velocity(vel_cmd)

    def _on_timeout_timer(self) -> None:
        """추적 결과가 끊기면 정지 명령을 발행한다."""
        if (
            not self._state_machine.is_active()
            or self._last_tracking_time is None
        ):
            return
        elapsed_sec = (
            self.get_clock().now() - self._last_tracking_time
        ).nanoseconds / 1_000_000_000
        if elapsed_sec > self._tracking_timeout_sec:
            self._publish_velocity(VelocityCommand())

    # 최종적으로 속도 계산하여 명령 전달
    def _publish_velocity(self, command: VelocityCommand) -> None:
        """속도 명령을 발행한다."""
        msg = Twist()
        msg.linear.x = command.linear_x
        msg.angular.z = command.angular_z
        self._velocity_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
