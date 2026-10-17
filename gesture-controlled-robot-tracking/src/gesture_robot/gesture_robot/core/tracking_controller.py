"""객체 위치를 로봇 이동 명령으로 변환하는 제어 영역."""

from dataclasses import dataclass


@dataclass
class VelocityCommand:
    """거북이 속도 명령."""
    linear_x: float = 0.0
    angular_z: float = 0.0


class TrackingController:
    """추적 상태와 객체 위치로 이동 명령을 계산한다."""

    def __init__(
        self,
        linear_gain: float = 0.004,
        angular_gain: float = 0.004,
        target_area: float = 2000.0,
        area_deadband: float = 100.0,
        angular_deadband: float = 10.0,
        max_linear_speed: float = 2.0,
        max_angular_speed: float = 2.0,
    ) -> None:
        self._linear_gain = linear_gain
        self._angular_gain = angular_gain
        self._target_area = target_area
        self._area_deadband = area_deadband
        self._angular_deadband = angular_deadband
        self._max_linear_speed = max_linear_speed
        self._max_angular_speed = max_angular_speed

    def calculate(
        self,
        tracking_active: bool,
        detected: bool,
        error_x: float,
        error_y: float,
        area: float,
    ) -> VelocityCommand:
        """현재 입력에 대응하는 속도 명령을 반환한다.

        Args:
            tracking_active: 추적 시작 상태 여부 (제스처 START 시 True)
            detected: 객체 검출 여부
            error_x: 화면 중심에서 객체까지의 수평 오차 (양수=우측)
            error_y: 화면 중심에서 객체까지의 수직 오차 (양수=하단)
            area: 검출된 객체의 면적 (px^2)

        Returns:
            VelocityCommand: 선속도(면적 비례), 각속도(위치 각도 비례)
        """
        if not tracking_active or not detected:
            return VelocityCommand(linear_x=0.0, angular_z=0.0)

        linear_x = self._calculate_linear_speed(area)
        angular_z = self._calculate_angular_speed(error_x, error_y)

        return VelocityCommand(
            linear_x=self._clamp(linear_x, 0.0, self._max_linear_speed),
            angular_z=self._clamp(
                angular_z,
                -self._max_angular_speed,
                self._max_angular_speed,
            ),
        )

    def _calculate_linear_speed(self, area: float) -> float:
        """목표 면적과 현재 면적 차이로 전진 속도를 계산한다."""
        area_error = self._target_area - area
        if area_error <= self._area_deadband:
            return 0.0
        return self._linear_gain * area_error

    def _calculate_angular_speed(self, error_x: float, error_y: float) -> float:
        """화면 중심 기준 가로 오차로 회전 속도를 계산한다."""
        del error_y
        if abs(error_x) <= self._angular_deadband:
            return 0.0
        return -self._angular_gain * error_x

    @staticmethod
    def _clamp(value: float, minimum: float, maximum: float) -> float:
        return max(minimum, min(maximum, value))
