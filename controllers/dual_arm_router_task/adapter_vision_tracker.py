"""手臂二的插座相機取樣、畫框與移動目標追蹤。"""

import math
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from statistics import median

from settings import adapter_vision as config
from adapter_vision import locate_adapter
from plan import POINTS, RUN_ORDER


@dataclass(frozen=True)
class LocalizedAdapter:
    """由連續相機影像確認的插座位置與時間。"""

    time: float
    x: float
    y: float
    z: float


class AdapterVisionTracker:
    """追蹤橘色插座開口，並在 Display 顯示 AOI 與辨識框。"""

    def __init__(self, robot, capture_start, capture_end, detector=locate_adapter):
        """初始化清潔頭上的相機與即時定位歷史。"""
        self.camera = robot.getDevice("adapter_socket_camera")
        self.camera.enable(config.CAMERA_PERIOD_MS)
        self.camera_node = robot.getFromDevice(self.camera._tag)
        self.display = robot.getDevice("adapter_socket_vision_display")
        self.display.attachCamera(self.camera)
        self.detector = detector
        self.capture_start = capture_start
        self.capture_end = capture_end
        self.point_id = RUN_ORDER[0]
        self.history = deque(maxlen=config.STABLE_FRAMES)
        self.latest = None
        self.last_sample_time = -1.0
        self._draw_overlay()

    def set_target(self, point_id, capture_start, capture_end):
        """開始新點位的追蹤，避免沿用上一站的影像。"""
        self.point_id = point_id
        self.capture_start = capture_start
        self.capture_end = capture_end
        self.history.clear()
        self.latest = None
        self.last_sample_time = -1.0
        self._draw_overlay()

    def sample(self, second):
        """接近插座時反覆拍照；連續定位穩定後更新三維位置。"""
        if not self.capture_start <= second <= self.capture_end:
            return
        if second - self.last_sample_time < config.SAMPLE_INTERVAL:
            return
        self.last_sample_time = second
        self._draw_overlay()
        detection = self.detector(
            self.camera.getImage(), self.camera_node.getPosition(), self.camera_node.getOrientation(),
        )
        if detection is None:
            self.history.clear()
            return
        x, y, z, box, confidence = detection
        if not self._valid_detection(x, y, z, confidence):
            self.history.clear()
            return
        self.history.append((x, y, z))
        self.display.setColor(0x00FFFF)
        self.display.drawRectangle(box[0], box[1], box[2] - box[0], box[3] - box[1])
        if len(self.history) == config.STABLE_FRAMES and all(
            max(axis) - min(axis) < config.MAX_FRAME_SPREAD for axis in zip(*self.history)
        ):
            axes = tuple(median(axis) for axis in zip(*self.history))
            self.latest = LocalizedAdapter(second, *axes)

    def recent(self, second):
        """回傳仍有效的插座位置；資料過期時回傳 None。"""
        if self.latest is None or second - self.latest.time > config.MAX_DETECTION_AGE:
            return None
        return self.latest

    def save_capture(self, directory):
        """儲存原始相機影像，方便檢查橘色外框與 AOI。"""
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        self.camera.saveImage(str(directory / f"{self.point_id}_adapter_socket_capture.png"), 100)

    def _valid_detection(self, x, y, z, confidence):
        """排除非有限值、低信心及超出工作區的外框。"""
        return (all(math.isfinite(value) for value in (x, y, z))
                and confidence >= config.MIN_CONFIDENCE
                and abs(x - POINTS[self.point_id][0]) < config.MAX_NOMINAL_X_ERROR
                and abs(y - POINTS[self.point_id][1]) < config.MAX_NOMINAL_Y_ERROR
                and abs(z - POINTS[self.point_id][2]) < config.MAX_NOMINAL_Z_ERROR)

    def _draw_overlay(self):
        """清除上一張標記，重新畫出綠色搜尋區域。"""
        self.display.setAlpha(0.0)
        self.display.fillRectangle(0, 0, config.CAMERA_WIDTH, config.CAMERA_HEIGHT)
        self.display.setAlpha(1.0)
        self.display.setColor(0x00FF00)
        left, top, right, bottom = config.AOI
        self.display.drawRectangle(left, top, right - left, bottom - top)
