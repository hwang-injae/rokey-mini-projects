"""HSV 기반 색상 객체 추적 알고리즘 영역.

이 모듈은 카메라 영상(BGR)을 HSV 색상 공간으로 변환하여
지정된 빨간색 공(객체)을 검출하고, 화면 중앙 대비 객체의 위치 오차(error_x, error_y) 및
면적(area)을 계산하는 핵심 추적 알고리즘을 담고 있습니다.
"""

from typing import Tuple, Optional, Dict, Any
import cv2
import numpy as np

from gesture_robot.core.models import TrackingResult


class ColorTracker:
    """OpenCV 기반 HSV 색상 검출기 클래스."""

    def __init__(
        self,
        min_area: float = 500.0,
        hue_min1: int = 0,
        hue_max1: int = 10,
        hue_min2: int = 170,
        hue_max2: int = 179,
        sat_min: int = 100,
        sat_max: int = 255,
        val_min: int = 100,
        val_max: int = 255,
        kernel_size: int = 5,
    ) -> None:
        """추적에 필요한 HSV 색상 범위와 모폴로지 커널 크기, 최소 면적 임계값을 설정합니다.

        Args:
            min_area: 객체로 인정할 최소 픽셀 면적 (기본값: 500.0 px²)
            hue_min1, hue_max1: 빨간색 첫 번째 Hue 구간 (0 ~ 10)
            hue_min2, hue_max2: 빨간색 두 번째 Hue 구간 (170 ~ 179)
            sat_min, sat_max: 채도 범위 (100 ~ 255)
            val_min, val_max: 명도 범위 (100 ~ 255)
            kernel_size: 모폴로지 노이즈 제거용 사각형 커널 크기 (기본값: 5x5)
        """
        self.min_area = min_area
        self.kernel_size = kernel_size

        # 1. 빨간색은 HSV Hue 원형 색상환에서 0번 부근과 180번 부근 2개 범위에 걸쳐 존재합니다.
        # 첫 번째 빨간색 범위 (예: 0~10도)
        self.lower_red_1 = np.array([hue_min1, sat_min, val_min], dtype=np.uint8)
        self.upper_red_1 = np.array([hue_max1, sat_max, val_max], dtype=np.uint8)
        
        # 두 번째 빨간색 범위 (예: 170~179도)
        self.lower_red_2 = np.array([hue_min2, sat_min, val_min], dtype=np.uint8)
        self.upper_red_2 = np.array([hue_max2, sat_max, val_max], dtype=np.uint8)

    def track(self, frame: np.ndarray) -> Tuple[TrackingResult, Dict[str, Any]]:
        """단일 영상 프레임에서 빨간색 객체를 추적하여 오차와 면적을 계산합니다.

        Step 1: BGR 영상을 HSV 색상 공간으로 변환
        Step 2: 2개 HSV 범위를 이용해 이진 마스크(Mask) 생성
        Step 3: 모폴로지 연산(Open/Close)으로 이미지 노이즈 제거
        Step 4: 윤곽선(Contour) 추출 및 가장 큰 객체 탐색
        Step 5: 객체 중심점(Moments) 및 화면 중앙 대비 오차(error_x, error_y) 계산

        Args:
            frame: OpenCV 카메라 프레임 (BGR 포맷)

        Returns:
            Tuple[TrackingResult, dict]: 
                - TrackingResult: ROS2 메시지 발행용 데이터 모델 (detected, error_x, error_y, area)
                - dict: 시각화/디버깅용 추가 정보 (center_x, center_y, mask, contour)
        """
        # Step 1. BGR -> HSV 변환 (조명 변화에 더 견고한 색상 추적 가능)
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Step 2. 지정한 HSV 범위에 해당하는 영역을 흰색(255), 나머지를 검은색(0) 마스크로 만듭니다.
        mask_1 = cv2.inRange(hsv_frame, self.lower_red_1, self.upper_red_1)
        mask_2 = cv2.inRange(hsv_frame, self.lower_red_2, self.upper_red_2)
        # 두 마스크를 비트OR 연산으로 하나로 합칩니다.
        mask = cv2.bitwise_or(mask_1, mask_2)

        # Step 3. 모폴로지(Morphology) 연산으로 자잘한 노이즈 제거 및 구멍 메우기
        kernel = np.ones((self.kernel_size, self.kernel_size), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)   # 열기: 자잘한 흰색 점 노이즈 제거
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # 닫기: 흰색 영역 내부의 작은 검은 구멍 메우기

        # Step 4. 이진 마스크에서 흰색 객체 윤곽선(Contours) 찾기
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        # 검출된 윤곽선이 없으면 미검출 처리
        if not contours:
            return self._not_detected(mask)

        # 검출된 여러 윤곽선 중 픽셀 면적(Area)이 가장 큰 외곽선을 선택
        largest_contour = max(contours, key=cv2.contourArea)
        area = float(cv2.contourArea(largest_contour))

        # 가장 큰 면적이 우리가 설정한 최소 면적(min_area)보다 작으면 노이즈로 판단하고 미검출 처리
        if area < self.min_area:
            return self._not_detected(mask)

        # Step 5. 모멘트(Moments)를 이용해 가장 큰 객체의 2차원 중심점(무게중심) 계산
        moments = cv2.moments(largest_contour)
        if moments["m00"] == 0:  # 0으로 나누는 예외 방지
            return self._not_detected(mask)

        center_x = int(moments["m10"] / moments["m00"])
        center_y = int(moments["m01"] / moments["m00"])

        # 카메라 전체 화면의 중앙 좌표 계산 (640x480 화면인 경우 중앙은 (320, 240))
        frame_height, frame_width = frame.shape[:2]
        frame_center_x = frame_width // 2
        frame_center_y = frame_height // 2

        # Step 6. 화면 중앙 대비 공 위치의 오차(Pixel Error) 산출 (SOT 확정 규칙 적용)
        # error_x: 공이 화면 중앙보다 오른쪽에 있으면 (+), 왼쪽에 있으면 (-)
        # error_y: 공이 화면 중앙보다 아래쪽에 있으면 (+), 위쪽에 있으면 (-)
        error_x = float(center_x - frame_center_x)
        error_y = float(center_y - frame_center_y)

        # ROS2 노드로 전달할 결과 모델 생성
        result = TrackingResult(
            detected=True,
            error_x=error_x,
            error_y=error_y,
            area=area,
        )

        # 시각화 패널이나 화면 표시용 디버그 데이터
        debug_info = {
            "center_x": center_x,
            "center_y": center_y,
            "mask": mask,
            "contour": largest_contour,
        }

        return result, debug_info

    @staticmethod
    def _not_detected(mask: np.ndarray) -> Tuple[TrackingResult, Dict[str, Any]]:
        """공이 검출되지 않았거나 면적이 너무 작을 때 반환하는 기본 객체 생성."""
        result = TrackingResult(
            detected=False,
            error_x=0.0,
            error_y=0.0,
            area=0.0,
        )
        debug_info = {
            "center_x": None,
            "center_y": None,
            "mask": mask,
            "contour": None,
        }
        return result, debug_info


