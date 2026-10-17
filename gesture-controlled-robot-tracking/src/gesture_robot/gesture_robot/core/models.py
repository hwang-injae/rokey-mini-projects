"""노드 내부에서 사용할 Python 데이터 모델 정의 영역."""

from enum import Enum
from dataclasses import dataclass


class GestureCommand(Enum):
    NONE = 0
    START = 1
    STOP = 2


class GestureResult:
    def __init__(self, command: GestureCommand, confidence: float):
        self.command = command
        self.confidence = confidence


@dataclass
class TrackingResult:
    detected: bool = False
    error_x: float = 0.0
    error_y: float = 0.0
    area: float = 0.0


class VelocityCommand:
    def __init__(self, linear_x: float, angular_z: float):
        self.linear_x = linear_x
        self.angular_z = angular_z
