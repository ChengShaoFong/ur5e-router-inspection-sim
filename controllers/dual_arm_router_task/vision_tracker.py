"""Webots 相機取樣、畫面標記與多張影像確認。"""

import math
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from statistics import median

from settings import port_vision as config
from plan import POINTS, RUN_ORDER
from port_vision import locate_port


@dataclass(frozen=True)
class LocalizedPort:
    """已由連續影像確認的 Port A 世界座標。"""

    time: float
    y: float
    z: float


class PortVisionTracker:
    """管理相機與 Display；更換辨識法時維持 locate_port 的回傳介面。"""

    def __init__(self, robot, capture_start, capture_end, detector=locate_port):
        """取得手臂相機、Router 與顯示器，初始化影像歷史。"""
        self.camera = robot.getDevice("port_a_camera")
        self.camera.enable(config.CAMERA_PERIOD_MS)
        self.detector = detector
        self.capture_start = capture_start
        self.capture_end = capture_end
        self.point_id = RUN_ORDER[0]
        self.camera_node = robot.getFromDevice(self.camera._tag)
        self.router_node = robot.getFromDef("ROUTER")
        if self.router_node is None:
            raise RuntimeError("ROUTER is missing from the world")
        self.display = robot.getDevice("port_a_vision_display")
        self.display.attachCamera(self.camera)
        self.history = deque(maxlen=config.STABLE_FRAMES)
        self.latest = None
        self.last_sample_time = -1.0
        self._draw_overlay()

    def set_target(self, point_id, capture_start, capture_end):
        """切換拍攝點位，清除前一站的定位結果。"""
        self.point_id = point_id
        self.capture_start = capture_start
        self.capture_end = capture_end
        self.history.clear()
        self.latest = None
        self.last_sample_time = -1.0
        self._draw_overlay()

    def sample(self, second):
        """僅在停穩拍照區間取樣，更新青色框與穩定定位結果。"""
        if not self.capture_start <= second <= self.capture_end:
            return
        if second - self.last_sample_time < config.SAMPLE_INTERVAL:
            return
        self.last_sample_time = second
        self._draw_overlay()
        detection = self.detector(
            self.camera.getImage(),
            self.camera_node.getPosition(),
            self.camera_node.getOrientation(),
            self.router_node.getPosition()[0] + config.RIM_FRONT_X_FROM_ROUTER,
        )
        if detection is None:
            self.history.clear()
            return
        world_y, world_z, box, confidence = detection
        if not self._valid_detection(world_y, world_z, confidence):
            self.history.clear()
            return

        self.history.append((world_y, world_z))
        self.display.setColor(0x00FFFF)
        self.display.drawRectangle(box[0], box[1], box[2] - box[0], box[3] - box[1])
        ys, zs = zip(*self.history)
        if (len(self.history) == config.STABLE_FRAMES
                and max(ys) - min(ys) < config.MAX_FRAME_SPREAD
                and max(zs) - min(zs) < config.MAX_FRAME_SPREAD):
            self.latest = LocalizedPort(second, median(ys), median(zs))

    def recent(self, second):
        """回傳仍有效的定位；過期時回傳 None。"""
        if self.latest is None or second - self.latest.time > config.MAX_DETECTION_AGE:
            return None
        return self.latest

    def save_capture(self, directory):
        """儲存未畫框的原始相機影像，供現場調整 AOI 與門檻。"""
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        self.camera.saveImage(str(directory / f"{self.point_id}_capture.png"), 100)

    def _valid_detection(self, y, z, confidence):
        """排除低信心、非有限值或遠離預定插孔的誤判。"""
        return (math.isfinite(y) and math.isfinite(z)
                and confidence >= config.MIN_CONFIDENCE
                and abs(y - POINTS[self.point_id][1]) < config.MAX_NOMINAL_ERROR
                and abs(z - POINTS[self.point_id][2]) < config.MAX_NOMINAL_ERROR)

    def _draw_overlay(self):
        """清除前一張標記，再於相機畫面顯示綠色 AOI。"""
        self.display.setAlpha(0.0)
        self.display.fillRectangle(0, 0, config.CAMERA_WIDTH, config.CAMERA_HEIGHT)
        self.display.setAlpha(1.0)
        self.display.setColor(0x00FF00)
        left, top, right, bottom = config.AOI
        self.display.drawRectangle(left, top, right - left, bottom - top)
