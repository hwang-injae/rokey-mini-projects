"""절대 좌표(pose) 기반으로 목표 지점을 추적하는 순수 계산 영역.

tracking_controller.py의 화면-오차 기반 제어(카메라가 로봇 자신에게 달려있다는
전제)와 달리, 이 모듈은 로봇의 실제 위치/방향과 목표 지점의 실제 좌표를 직접
비교해 추적 속도를 계산한다. turtlesim(Pose)과 Gazebo(Odometry) 양쪽에서
공통으로 재사용한다.
"""

import math
from dataclasses import dataclass


@dataclass
class PursuitVelocity:
    """로봇 속도 명령."""
    linear_x: float = 0.0
    angular_z: float = 0.0


class PursuitController:
    """로봇의 현재 pose와 목표 좌표로 추적 속도를 계산한다."""

    def __init__(
        self,
        linear_gain: float = 1.5,
        angular_gain: float = 4.0,
        max_linear_speed: float = 2.0,
        max_angular_speed: float = 3.0,
        distance_tolerance: float = 0.3,
        heading_lock_deg: float = 45.0,
    ) -> None:
        self._linear_gain = linear_gain
        self._angular_gain = angular_gain
        self._max_linear_speed = max_linear_speed
        self._max_angular_speed = max_angular_speed
        self._distance_tolerance = distance_tolerance
        self._heading_lock_rad = math.radians(heading_lock_deg)

    def calculate(
        self,
        robot_x: float,
        robot_y: float,
        robot_theta: float,
        target_x: float,
        target_y: float,
    ) -> PursuitVelocity:
        """robot이 target을 향하도록 필요한 속도를 계산한다."""
        dx = target_x - robot_x
        dy = target_y - robot_y
        distance = math.hypot(dx, dy)

        if distance <= self._distance_tolerance:
            return PursuitVelocity()

        desired_theta = math.atan2(dy, dx)
        heading_error = self._normalize_angle(desired_theta - robot_theta)

        angular_z = self._clamp(
            self._angular_gain * heading_error,
            -self._max_angular_speed,
            self._max_angular_speed,
        )

        linear_x = 0.0
        # 방향이 크게 어긋나 있으면(옆/뒤에 있으면) 제자리 회전부터 하고,
        # 어느 정도 정면을 향한 뒤에 전진한다.
        if abs(heading_error) < self._heading_lock_rad:
            linear_x = self._clamp(
                self._linear_gain * distance, 0.0, self._max_linear_speed
            )

        return PursuitVelocity(linear_x=linear_x, angular_z=angular_z)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    @staticmethod
    def _clamp(value: float, minimum: float, maximum: float) -> float:
        return max(minimum, min(maximum, value))
