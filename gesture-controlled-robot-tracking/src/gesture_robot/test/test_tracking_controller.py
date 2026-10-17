"""TrackingController 이동 제어 단위 테스트."""

import pytest

from gesture_robot.core.tracking_controller import TrackingController


def test_target_area_stops_only_forward_motion():
    """목표 면적이면 전진은 멈추고 좌우 보정 회전은 유지한다."""
    controller = TrackingController(
        linear_gain=0.001,
        angular_gain=0.01,
        target_area=2000.0,
        area_deadband=100.0,
        angular_deadband=10.0,
        max_linear_speed=2.0,
        max_angular_speed=2.0,
    )

    command = controller.calculate(
        tracking_active=True,
        detected=True,
        error_x=50.0,
        error_y=0.0,
        area=2000.0,
    )

    assert command.linear_x == 0.0
    assert command.angular_z == pytest.approx(-0.5)


def test_oversized_object_does_not_reverse():
    """목표보다 공이 크게 보여도 후진하지 않는다."""
    controller = TrackingController(
        linear_gain=0.001,
        angular_gain=0.01,
        target_area=2000.0,
        area_deadband=100.0,
        angular_deadband=10.0,
        max_linear_speed=2.0,
        max_angular_speed=2.0,
    )

    command = controller.calculate(
        tracking_active=True,
        detected=True,
        error_x=-50.0,
        error_y=0.0,
        area=2500.0,
    )

    assert command.linear_x == 0.0
    assert command.angular_z == pytest.approx(0.5)


def test_stop_state_or_not_detected_publishes_zero_velocity():
    """STOP 상태나 객체 미검출이면 전진과 회전을 모두 멈춘다."""
    controller = TrackingController(angular_gain=0.01)

    stopped = controller.calculate(
        tracking_active=False,
        detected=True,
        error_x=50.0,
        error_y=0.0,
        area=1000.0,
    )
    not_detected = controller.calculate(
        tracking_active=True,
        detected=False,
        error_x=50.0,
        error_y=0.0,
        area=1000.0,
    )

    assert stopped.linear_x == 0.0
    assert stopped.angular_z == 0.0
    assert not_detected.linear_x == 0.0
    assert not_detected.angular_z == 0.0
