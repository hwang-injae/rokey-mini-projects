"""시뮬레이터 연동 ROS2 노드 골격."""

from typing import Any


class SimulationNode:
    """이동 명령을 시뮬레이터에 전달한다."""

    def __init__(self, simulator: Any) -> None:
        # TODO: rclpy Node 상속, 어댑터 저장, 속도 Subscriber 생성을 구현한다.
        pass

    def handle_velocity_command(self, velocity_command: Any) -> None:
        """수신한 이동 명령을 처리한다."""
        # TODO: 이동 명령을 시뮬레이터 어댑터에 전달한다.
        pass
