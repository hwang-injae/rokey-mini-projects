"""추적 시작과 정지 상태를 관리하는 영역."""

from enum import Enum


class TrackingState(Enum):
    """추적 상태."""
    STOP = "STOP"
    START = "START"


class TrackingStateMachine:
    """제스처 명령에 따라 현재 추적 상태를 관리한다."""

    NONE = 0
    START = 1
    STOP = 2

    def __init__(self) -> None:
        self._state = TrackingState.STOP

    def update(self, command: int) -> None:
        """제스처 명령으로 상태를 변경한다."""
        if command == self.START:
            self._state = TrackingState.START
        elif command == self.STOP:
            self._state = TrackingState.STOP
        # NONE 등 기타 명령은 상태 유지

    def is_active(self) -> bool:
        """현재 추적 시작 상태인지 반환한다."""
        return self._state == TrackingState.START

    def current_state(self) -> str:
        """현재 추적 상태를 반환한다."""
        return self._state.value
