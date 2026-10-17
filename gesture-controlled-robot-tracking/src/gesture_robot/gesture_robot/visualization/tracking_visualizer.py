"""OpenCV 기반 추적 결과 시각화 영역."""

import cv2


class TrackingVisualizer:
    """원본 영상 위에 추적 결과와 상태를 표시한다."""

    def render(self, frame, tracking_result, debug_info=None):
        """영상 위에 객체 위치, 바운딩 박스, 상태 텍스트를 그린다 (frame을 직접 수정)."""
        if tracking_result.detected and debug_info is not None:
            contour = debug_info.get("contour")
            if contour is not None:
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            center_x = debug_info.get("center_x")
            center_y = debug_info.get("center_y")
            if center_x is not None and center_y is not None:
                cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)

        if tracking_result.detected:
            status_text = (
                f"area={tracking_result.area:.0f} "
                f"err=({tracking_result.error_x:+.0f},{tracking_result.error_y:+.0f})"
            )
        else:
            status_text = "NOT DETECTED"

        cv2.putText(
            frame, status_text, (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA,
        )
        return frame
