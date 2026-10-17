"""MediaPipe 손 랜드마크 검출 연결 영역."""

import mediapipe as mp
from gesture_robot.core.models import GestureCommand, GestureResult

class MediaPipeDetector:
    """영상 프레임에서 손 랜드마크를 검출한다."""
    def __init__(self, max_num_hands=1, min_detection_confidence=0.7, min_tracking_confidence=0.5):
        self._hands = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=max_num_hands,
            min_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence,
        )
    def detect(self, frame):
        """검출한 손 랜드마크를 반환한다."""
        # TODO: MediaPipe 초기화와 손 랜드마크 검출을 구현한다.
        import cv2
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = self._hands.process(rgb)

        if not result.multi_hand_landmarks:
            return GestureResult(command=GestureCommand.NONE, confidence=0.0)
        return result.multi_hand_landmarks[0]
