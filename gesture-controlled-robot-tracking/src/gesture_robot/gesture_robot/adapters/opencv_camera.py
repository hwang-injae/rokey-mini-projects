"""OpenCV 카메라 입력 연결 영역."""

from typing import Optional

import cv2
import numpy as np


class OpenCVCamera:
    """카메라 장치로부터 영상 프레임을 제공한다."""

    def __init__(self, device_id: int = 0) -> None:
        self._device_id = device_id
        self._capture: Optional[cv2.VideoCapture] = None

    def open(self) -> bool:
        """카메라 장치를 연다."""
        self._capture = cv2.VideoCapture(self._device_id)
        return self._capture.isOpened()

    def read(self) -> Optional[np.ndarray]:
        """카메라에서 한 프레임을 읽는다."""
        if self._capture is None:
            return None
        ok, frame = self._capture.read()
        return frame if ok else None

    def release(self) -> None:
        """카메라 자원을 해제한다."""
        if self._capture is not None:
            self._capture.release()
            self._capture = None
