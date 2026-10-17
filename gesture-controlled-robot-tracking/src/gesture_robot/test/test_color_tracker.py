"""ColorTracker 핵심 알고리즘 단위 테스트 모듈.

3주차 회의록에서 확정한 4가지 최소 검증 기준:
1. 빨간색 객체 정상 검출
2. 빈 이미지(빨간색 없음) 미검출 처리
3. 최소 면적(500.0 px²) 미만 소형 객체 미검출 처리
4. 화면 중앙 기준 위치 오차(error_x, error_y) 부호 정합성 검증
"""

import cv2
import numpy as np
import pytest

from gesture_robot.core.color_tracker import ColorTracker


def create_blank_image(width=640, height=480, color=(0, 0, 0)):
    """지정한 크기와 BGR 색상의 단색 이미지를 생성합니다."""
    image = np.zeros((height, width, 3), dtype=np.uint8)
    image[:] = color
    return image


def test_red_object_detection():
    """1. 빨간색 객체(공)가 있을 때 정상적으로 검출되는지 검증합니다."""
    tracker = ColorTracker(min_area=500.0)
    image = create_blank_image(640, 480)

    # 화면 중앙 부근(320, 240)에 반지름 40px인 빨간색 원 그리기
    # BGR 포맷에서 빨간색은 (0, 0, 255)
    cv2.circle(image, (320, 240), 40, (0, 0, 255), -1)

    result, debug_info = tracker.track(image)

    assert result.detected is True
    assert result.area > 500.0
    assert pytest.approx(result.error_x, abs=5.0) == 0.0
    assert pytest.approx(result.error_y, abs=5.0) == 0.0


def test_empty_image_not_detected():
    """2. 빨간색 객체가 없는 빈(또는 다른 색상) 이미지일 때 미검출 처리되는지 검증합니다."""
    tracker = ColorTracker(min_area=500.0)
    
    # 검은색 이미지
    black_image = create_blank_image(640, 480, color=(0, 0, 0))
    result, _ = tracker.track(black_image)
    assert result.detected is False
    assert result.area == 0.0
    assert result.error_x == 0.0

    # 파란색 이미지
    blue_image = create_blank_image(640, 480, color=(255, 0, 0))
    result, _ = tracker.track(blue_image)
    assert result.detected is False


def test_small_area_below_threshold():
    """3. 빨간색 객체가 있지만 최소 면적(500.0 px²) 미만일 때 미검출 처리되는지 검증합니다."""
    tracker = ColorTracker(min_area=500.0)
    image = create_blank_image(640, 480)

    # 반지름 5px 인 아주 작은 빨간 점 (면적 약 78.5 px² < 500 px²)
    cv2.circle(image, (320, 240), 5, (0, 0, 255), -1)

    result, _ = tracker.track(image)

    # 면적이 threshold 미만이므로 detected가 False이어야 함
    assert result.detected is False


def test_error_sign_conventions():
    """4. 화면 중앙 대비 위치 오차(error_x, error_y)의 부호 규칙을 검증합니다.
    
    SOT 부호 규칙:
    - 오른쪽: error_x > 0
    - 왼쪽: error_x < 0
    - 아래쪽: error_y > 0
    - 위쪽: error_y < 0
    """
    tracker = ColorTracker(min_area=500.0)

    # (A) 화면 오른쪽(450, 240)에 공이 있는 경우 -> error_x > 0
    image_right = create_blank_image(640, 480)
    cv2.circle(image_right, (450, 240), 30, (0, 0, 255), -1)
    res_right, _ = tracker.track(image_right)
    assert res_right.detected is True
    assert res_right.error_x > 0  # 450 - 320 = +130

    # (B) 화면 왼쪽(150, 240)에 공이 있는 경우 -> error_x < 0
    image_left = create_blank_image(640, 480)
    cv2.circle(image_left, (150, 240), 30, (0, 0, 255), -1)
    res_left, _ = tracker.track(image_left)
    assert res_left.detected is True
    assert res_left.error_x < 0  # 150 - 320 = -170

    # (C) 화면 아래쪽(320, 380)에 공이 있는 경우 -> error_y > 0
    image_down = create_blank_image(640, 480)
    cv2.circle(image_down, (320, 380), 30, (0, 0, 255), -1)
    res_down, _ = tracker.track(image_down)
    assert res_down.detected is True
    assert res_down.error_y > 0  # 380 - 240 = +140

    # (D) 화면 위쪽(320, 100)에 공이 있는 경우 -> error_y < 0
    image_up = create_blank_image(640, 480)
    cv2.circle(image_up, (320, 100), 30, (0, 0, 255), -1)
    res_up, _ = tracker.track(image_up)
    assert res_up.detected is True
    assert res_up.error_y < 0  # 100 - 240 = -140
