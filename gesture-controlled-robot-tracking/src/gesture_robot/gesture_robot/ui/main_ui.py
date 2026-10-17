"""Tkinter 기반 메인 UI 영역."""

import time
from collections import deque
from typing import Optional

import cv2
import numpy as np
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk


class MainUI:
    _TITLE = "Gesture-Controlled Tracking Robot"
    _WIDTH = 1300
    _HEIGHT = 700
    _CAM_W = 640
    _CAM_H = 360
    _SIM_H = 200
    _SIDE_W = 480
    _FLOW_H = 220
    _VEL_H = 220
    _VEL_MAXLEN = 100
    _TICK_MS = 50

    _GESTURE_NAMES = {0: "NONE", 1: "START", 2: "STOP"}

    # SOT 확정 토픽 계약 (docs/SOT.md, 3주차 회의록 참고)
    _TOPICS = [
        "camera/image_raw",
        "gesture/command",
        "tracking/object",
        "turtle1/cmd_vel",
    ]

    def __init__(self, use_mock: bool = True) -> None:
        self.root = tk.Tk()
        self.root.title(self._TITLE)
        self.root.geometry(f"{self._WIDTH}x{self._HEIGHT}")
        self.root.resizable(False, False)

        self._cam_photo: Optional[ImageTk.PhotoImage] = None
        self._sim_photo: Optional[ImageTk.PhotoImage] = None
        self._linear_history: deque[float] = deque(maxlen=self._VEL_MAXLEN)
        self._angular_history: deque[float] = deque(maxlen=self._VEL_MAXLEN)
        self._topic_last_seen: dict[str, float] = {}

        self._build_ui()
        if use_mock:
            self._mock_start()

    # --- Build UI ---

    def _build_ui(self) -> None:
        left = ttk.Frame(self.root)
        left.pack(side="left", fill="both", expand=True)
        right = ttk.Frame(self.root)
        right.pack(side="left", fill="both", expand=True)

        self._build_camera(left)
        self._build_status(left)
        self._build_simulation(left)
        self._build_topic_flow(right)
        self._build_velocity(right)

    def _build_camera(self, parent: tk.Widget) -> None:
        frame = ttk.LabelFrame(parent, text="CAMERA", padding=5)
        frame.pack(fill="both", expand=True, padx=10, pady=(10, 5))

        self._cam_canvas = tk.Canvas(
            frame, width=self._CAM_W, height=self._CAM_H,
            bg="black", highlightthickness=0,
        )
        self._cam_canvas.pack()
        self._cam_canvas.create_text(
            self._CAM_W // 2, self._CAM_H // 2,
            text="CAMERA FEED", fill="#444", font=("Arial", 18),
        )

    def _build_status(self, parent: tk.Widget) -> None:
        frame = ttk.LabelFrame(parent, text="STATUS", padding=10)
        frame.pack(fill="x", padx=10, pady=5)

        self._svars: dict[str, tk.StringVar] = {}
        fields = [
            ("Tracking", "tracking", 8),
            ("Gesture", "gesture", 8),
            ("Confidence", "confidence", 6),
            ("Error X", "error_x", 8),
            ("Error Y", "error_y", 8),
            ("Area", "area", 6),
            ("FPS", "fps", 6),
        ]

        for col, (label, key, width) in enumerate(fields):
            self._svars[key] = tk.StringVar(value="--")
            ttk.Label(frame, text=label).grid(
                row=0, column=col * 2, sticky="w", padx=(5, 2),
            )
            ttk.Label(frame, textvariable=self._svars[key], width=width, anchor="w").grid(
                row=0, column=col * 2 + 1, sticky="w", padx=(0, 10),
            )

    def _build_simulation(self, parent: tk.Widget) -> None:
        frame = ttk.LabelFrame(parent, text="SIMULATION", padding=5)
        frame.pack(fill="x", padx=10, pady=(5, 10))

        self._sim_canvas = tk.Canvas(
            frame, width=self._CAM_W, height=self._SIM_H,
            bg="#f0f0f0", highlightthickness=0,
        )
        self._sim_canvas.pack()
        self._sim_canvas.create_text(
            self._CAM_W // 2, self._SIM_H // 2,
            text="TURTLESIM", fill="#ccc", font=("Arial", 20),
        )

    def _build_topic_flow(self, parent: tk.Widget) -> None:
        frame = ttk.LabelFrame(parent, text="TOPIC FLOW", padding=5)
        frame.pack(fill="x", padx=10, pady=(10, 5))

        self._flow_canvas = tk.Canvas(
            frame, width=self._SIDE_W, height=self._FLOW_H,
            bg="#fafafa", highlightthickness=0,
        )
        self._flow_canvas.pack()

        row_h = self._FLOW_H / len(self._TOPICS)
        self._flow_dots: list[int] = []
        for i, topic in enumerate(self._TOPICS):
            y = row_h * i + row_h / 2
            dot = self._flow_canvas.create_oval(
                20, y - 8, 36, y + 8, fill="#ccc", outline="",
            )
            self._flow_canvas.create_text(
                50, y, text=topic, anchor="w", font=("Consolas", 11),
            )
            self._flow_dots.append(dot)

    def _build_velocity(self, parent: tk.Widget) -> None:
        frame = ttk.LabelFrame(parent, text="VELOCITY", padding=5)
        frame.pack(fill="x", padx=10, pady=(5, 10))

        self._vel_canvas = tk.Canvas(
            frame, width=self._SIDE_W, height=self._VEL_H,
            bg="#ffffff", highlightthickness=0,
        )
        self._vel_canvas.pack()
        mid_y = self._VEL_H // 2
        self._vel_canvas.create_line(
            0, mid_y, self._SIDE_W, mid_y, fill="#ddd",
        )
        self._vel_canvas.create_text(
            10, 10, anchor="nw", fill="#3a7", font=("Arial", 9),
            text="linear_x",
        )
        self._vel_canvas.create_text(
            10, 26, anchor="nw", fill="#c33", font=("Arial", 9),
            text="angular_z",
        )

    # --- Public API (2단계에서 ROS2 Subscribe 시 호출) ---

    def update_camera_frame(self, frame: np.ndarray) -> None:
        """OpenCV BGR 프레임을 CAMERA 영역에 표시한다."""
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        pil_img = Image.fromarray(rgb)
        pil_img = pil_img.resize((self._CAM_W, self._CAM_H), Image.LANCZOS)
        self._cam_photo = ImageTk.PhotoImage(pil_img)
        self._cam_canvas.delete("all")
        self._cam_canvas.create_image(
            self._CAM_W // 2, self._CAM_H // 2,
            image=self._cam_photo,
        )

    def update_status(
        self,
        tracking: str,
        gesture: int,
        confidence: float,
        error_x: float,
        error_y: float,
        area: float,
        fps: float,
    ) -> None:
        """모니터링 상태 정보를 갱신한다."""
        self._svars["tracking"].set(tracking)
        self._svars["gesture"].set(
            self._GESTURE_NAMES.get(gesture, f"UNKNOWN({gesture})"),
        )
        self._svars["confidence"].set(f"{confidence:.2f}")
        self._svars["error_x"].set(f"{error_x:+.1f}")
        self._svars["error_y"].set(f"{error_y:+.1f}")
        self._svars["area"].set(f"{area:.0f}")
        self._svars["fps"].set(f"{fps:.1f}")

    def update_topic_activity(self, active_topic: str) -> None:
        """토픽의 수신 타임스탬프를 기록하고, 최근 1초 내 수신된 모든 활성 토픽의 표시등을 켠다."""
        now = time.time()
        self._topic_last_seen[active_topic] = now
        for topic, dot in zip(self._TOPICS, self._flow_dots):
            last_seen = self._topic_last_seen.get(topic, 0.0)
            color = "#3a7" if (now - last_seen < 1.0) else "#ccc"
            self._flow_canvas.itemconfig(dot, fill=color)

    def update_velocity(self, linear_x: float, angular_z: float) -> None:
        """최신 속도 명령을 VELOCITY 그래프에 추가한다."""
        self._linear_history.append(linear_x)
        self._angular_history.append(angular_z)
        self._redraw_velocity()

    def _redraw_velocity(self) -> None:
        canvas = self._vel_canvas
        canvas.delete("series")
        mid_y = self._VEL_H // 2
        step = self._SIDE_W / self._VEL_MAXLEN
        scale = 80.0

        for history, color in (
            (self._linear_history, "#3a7"),
            (self._angular_history, "#c33"),
        ):
            if len(history) < 2:
                continue
            points = []
            start_x = self._SIDE_W - len(history) * step
            for i, value in enumerate(history):
                x = start_x + i * step
                y = mid_y - value * scale
                points.extend((x, y))
            canvas.create_line(*points, fill=color, width=2, tags="series")

    # --- Standalone mock (이번 주 테스트용, 2단계에서 제거) ---

    def _mock_start(self) -> None:
        self._mock_t = 0
        self._tick()

    def _tick(self) -> None:
        self._mock_t = (self._mock_t + 1) % 150

        t = self._mock_t
        if t < 50:
            tracking, gesture, conf, ex, ey, area, fps = (
                "STOP", 0, 0.0, 0.0, 0.0, 0.0, 30.0
            )
        elif t < 100:
            tracking, gesture, conf, ex, ey, area, fps = (
                "START", 1, 0.95, 42.0, -18.0, 1234.0, 30.0
            )
        else:
            tracking, gesture, conf, ex, ey, area, fps = (
                "STOP", 2, 0.88, 0.0, 0.0, 0.0, 30.0
            )

        self.update_status(tracking, gesture, conf, ex, ey, area, fps)
        self._mock_tick_topic_flow(t)
        self._mock_tick_velocity(tracking, ex, area)
        self.root.after(self._TICK_MS, self._tick)

    def _mock_tick_topic_flow(self, t: int) -> None:
        """토픽 흐름을 데이터 순서(카메라→인식/추적→제어→시뮬레이터)로 순환 표시한다."""
        active_index = (t // 5) % len(self._TOPICS)
        self.update_topic_activity(self._TOPICS[active_index])

    def _mock_tick_velocity(self, tracking: str, error_x: float, area: float) -> None:
        """3주차 회의록에서 확정한 제어식으로 모의 속도값을 계산한다."""
        if tracking == "STOP":
            linear_x, angular_z = 0.0, 0.0
        else:
            target_area, linear_gain, angular_gain = 1500.0, 0.003, 0.01
            linear_x = linear_gain * (target_area - area)
            angular_z = -angular_gain * error_x
        self.update_velocity(linear_x, angular_z)

    # --- Run ---

    def run(self) -> None:
        """UI 애플리케이션을 실행한다."""
        self.root.mainloop()


def main() -> None:
    app = MainUI()
    app.run()


if __name__ == "__main__":
    main()
