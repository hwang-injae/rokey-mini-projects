"""손 랜드마크를 제스처 명령으로 분류하는 영역."""

from gesture_robot.core.models import GestureCommand, GestureResult

class GestureClassifier:
    """MediaPipe 랜드마크를 프로젝트 명령으로 변환한다."""

    def classify(self, hand_landmarks) -> GestureResult:
        if isinstance(hand_landmarks, GestureResult):
            return hand_landmarks
        lm = hand_landmarks.landmark
        
        # 엄지 : TIP(4)과 IP(3) 사이 거리로 판별 (x-y 평면)
        thumb_tip = lm[4]
        thumb_ip = lm[3]
        thumb_open = ((thumb_tip.x - thumb_ip.x) ** 2 +
                      (thumb_tip.y -thumb_ip.y) ** 2) ** 0.5 > 0.05
        
        # 검지(8/6), 중지(12/10), 약지(16/14), 새끼(20/18): tip.y < pip.y 면 폄
        fingers = [(8,6), (12,10), (16,14), (20, 18)]
        open_count = 1 if thumb_open else 0
        # if thumb_open:
        #     open_count = 1
        # else:
        #     open_count = 0
        for tip_id, pip_id in fingers:
            if lm[tip_id].y < lm[pip_id].y:
                open_count += 1
        if open_count >= 4:
            return GestureResult(command=GestureCommand.START, confidence=0.9)
        elif open_count <= 1:
            return GestureResult(command=GestureCommand.STOP, confidence=0.9)
        else:
            return GestureResult(command=GestureCommand.NONE, confidence=0.5)
        
