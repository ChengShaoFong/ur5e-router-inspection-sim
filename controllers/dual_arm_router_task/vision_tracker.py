"""兩支相機共用取樣、畫框與連續影像確認；辨識演算法各自獨立。"""

import math
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from statistics import median

from adapter_vision import locate_adapter
from plan import POINTS, RUN_ORDER
from port_vision import locate_port
from settings import adapter_vision, port_vision


@dataclass(frozen=True)
class LocalizedPort:
    """插孔 y/z 世界座標及拍攝時間。"""

    time: float
    y: float
    z: float


@dataclass(frozen=True)
class LocalizedAdapter:
    """插座 x/y/z 世界座標及拍攝時間。"""

    time: float
    x: float
    y: float
    z: float


class _VisionTracker:
    """處理相機時窗、穩定定位與 Webots Display。"""

    def __init__(self, robot, capture_start, capture_end, settings, camera_name,
                 display_name, axes, nominal_errors, result_type, image_suffix):
        self.settings = settings
        self.camera = robot.getDevice(camera_name)
        self.camera.enable(settings.CAMERA_PERIOD_MS)
        self.camera_node = robot.getFromDevice(self.camera._tag)
        self.display = robot.getDevice(display_name)
        self.display.attachCamera(self.camera)
        self.axes = axes
        self.nominal_errors = nominal_errors
        self.result_type = result_type
        self.image_suffix = image_suffix
        self.history = deque(maxlen=settings.STABLE_FRAMES)
        self.point_id = RUN_ORDER[0]
        self.set_target(self.point_id, capture_start, capture_end)

    def set_target(self, point_id, capture_start, capture_end):
        """切換點位及拍攝時間，清除前一站的定位結果。"""
        self.point_id = point_id
        self.capture_start = capture_start
        self.capture_end = capture_end
        self.history.clear()
        self.latest = None
        self.last_sample_time = -1.0
        self._draw_overlay()

    def sample(self, second):
        """在指定時窗拍照，取得穩定目標後顯示青色偵測框。"""
        config = self.settings
        if not self.capture_start <= second <= self.capture_end:
            return
        if second - self.last_sample_time < config.SAMPLE_INTERVAL:
            return
        self.last_sample_time = second
        self._draw_overlay()
        detection = self._detect()
        if detection is None:
            self.history.clear()
            return
        *coordinates, box, confidence = detection
        nominal = POINTS[self.point_id]
        if (confidence < config.MIN_CONFIDENCE
                or any(not math.isfinite(value) or abs(value - nominal[axis]) >= error
                       for value, axis, error in zip(coordinates, self.axes, self.nominal_errors))):
            self.history.clear()
            return
        self.history.append(tuple(coordinates))
        self.display.setColor(0x00FFFF)
        self.display.drawRectangle(box[0], box[1], box[2] - box[0], box[3] - box[1])
        if len(self.history) == config.STABLE_FRAMES and all(
            max(axis_values) - min(axis_values) < config.MAX_FRAME_SPREAD
            for axis_values in zip(*self.history)
        ):
            self.latest = self.result_type(second, *(median(axis_values) for axis_values in zip(*self.history)))

    def recent(self, second):
        """只回傳仍在有效時間內的定位。"""
        if self.latest is None or second - self.latest.time > self.settings.MAX_DETECTION_AGE:
            return None
        return self.latest

    def save_capture(self, directory):
        """儲存原始相機影像，供現場檢查 AOI。"""
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        self.camera.saveImage(str(directory / f"{self.point_id}{self.image_suffix}.png"), 100)

    def _draw_overlay(self):
        """清除前一張標記並重新畫出綠色 AOI。"""
        config = self.settings
        self.display.setAlpha(0.0)
        self.display.fillRectangle(0, 0, config.CAMERA_WIDTH, config.CAMERA_HEIGHT)
        self.display.setAlpha(1.0)
        self.display.setColor(0x00FF00)
        left, top, right, bottom = config.AOI
        self.display.drawRectangle(left, top, right - left, bottom - top)


class PortVisionTracker(_VisionTracker):
    """使用手臂一相機定位 Router 插孔。"""

    def __init__(self, robot, capture_start, capture_end, detector=locate_port):
        self.detector = detector
        self.router_node = robot.getFromDef("ROUTER")
        if self.router_node is None:
            raise RuntimeError("ROUTER is missing from the world")
        super().__init__(robot, capture_start, capture_end, port_vision,
                         "port_a_camera", "port_a_vision_display", (1, 2),
                         (port_vision.MAX_NOMINAL_ERROR,) * 2, LocalizedPort, "_capture")

    def _detect(self):
        """把相機影像投影到插孔前平面。"""
        return self.detector(
            self.camera.getImage(), self.camera_node.getPosition(),
            self.camera_node.getOrientation(),
            self.router_node.getPosition()[0] + port_vision.RIM_FRONT_X_FROM_ROUTER,
        )


class AdapterVisionTracker(_VisionTracker):
    """使用手臂二相機追蹤插座開口。"""

    def __init__(self, robot, capture_start, capture_end, detector=locate_adapter):
        self.detector = detector
        super().__init__(robot, capture_start, capture_end, adapter_vision,
                         "adapter_socket_camera", "adapter_socket_vision_display", (0, 1, 2),
                         (adapter_vision.MAX_NOMINAL_X_ERROR,
                          adapter_vision.MAX_NOMINAL_Y_ERROR,
                          adapter_vision.MAX_NOMINAL_Z_ERROR),
                         LocalizedAdapter, "_adapter_socket_capture")

    def _detect(self):
        """從插座外框估計三維位置。"""
        return self.detector(
            self.camera.getImage(), self.camera_node.getPosition(),
            self.camera_node.getOrientation(),
        )
