"""Webots Display 上的三軸工具負載示波器（感測器局部座標、N）。"""

from collections import deque
import math


class ForceScope:
    """每個控制步取樣，限速重畫；保留原始尖峰及最近十秒波形。"""

    WINDOW_SECONDS = 10.0
    DRAW_INTERVAL = 0.1
    COLORS = (0xFF7474, 0x66DB9A, 0x6AB5FF, 0xFFD166)
    LABELS = ("Fx", "Fy", "Fz", "|F|")

    def __init__(self, robot, role, period_ms):
        self.sensor = robot.getDevice("tool_force_sensor")
        self.display = robot.getDevice("force_scope_display")
        if self.sensor is None or self.display is None:
            raise RuntimeError("World requires tool_force_sensor and force_scope_display")
        self.sensor.enable(period_ms)
        self.title = "ARM 1 / GRIPPER" if role == "arm1" else "ARM 2 / CLEANER"
        self.samples = deque(maxlen=math.ceil(self.WINDOW_SECONDS * 1000 / period_ms) + 2)
        self.last_draw = -math.inf
        self.width = self.display.getWidth()
        self.height = self.display.getHeight()
        self.display.setFont("Lucida Console", 12, True)

    def sample(self, second):
        """無效讀值以斷線呈現，避免 NaN 被當成零力或造成繪圖錯誤。"""
        # R2025a returns a ctypes pointer without a length, not a Python list.
        # Iterating it directly reads past the three doubles and can crash Python.
        values = tuple(self.sensor.getValues()[:3])
        magnitude = math.hypot(*values)
        valid = len(values) == 3 and all(math.isfinite(x) for x in values) and math.isfinite(magnitude)
        if self.samples and second < self.samples[-1][0]:
            self.samples.clear()
            self.last_draw = -math.inf
        self.samples.append((second, values + (magnitude,) if valid else None))
        while self.samples and self.samples[0][0] < second - self.WINDOW_SECONDS:
            self.samples.popleft()
        if second - self.last_draw >= self.DRAW_INTERVAL:
            self._draw(second)
            self.last_draw = second

    def _text(self, text, x, y, color=0xC5D1DF):
        self.display.setColor(color)
        self.display.drawText(text, x, y)

    def _draw(self, second):
        display = self.display
        left, right, top, bottom = 65, self.width - 18, 82, self.height - 40
        peak = max((values[3] for _, values in self.samples if values is not None), default=0.0)
        # Round up to a stable 1/2/5 scale; always leave headroom for the peak.
        target = max(1.0, peak * 1.15)
        power = 10 ** math.floor(math.log10(target))
        scale = next(factor * power for factor in (1, 2, 5, 10) if factor * power >= target)
        display.setColor(0x101923)
        display.fillRectangle(0, 0, self.width, self.height)
        self._text(self.title + "   FORCE SCOPE", 12, 10, 0xFFFFFF)
        self._text(f"LOCAL XYZ / N   RAW LOAD   t={second:.2f}s", 12, 29)
        latest = self.samples[-1][1]
        for channel, (label, color) in enumerate(zip(self.LABELS, self.COLORS)):
            value = f"{latest[channel]:.2f}" if latest is not None else "N/A"
            self._text(f"{label} {value}", 12 + channel * (self.width // 4), 51, color)

        for index in range(5):
            y = round(top + index * (bottom - top) / 4)
            display.setColor(0x526478 if index == 2 else 0x283747)
            display.drawLine(left, y, right, y)
            self._text(f"{scale * (1 - index / 2):.3g}", 4, y - 6)
        start = second - self.WINDOW_SECONDS
        for index in range(6):
            x = round(left + index * (right - left) / 5)
            display.setColor(0x283747)
            display.drawLine(x, top, x, bottom)
            self._text(f"{start + index * 2:.1f}", x - 14, bottom + 8)

        for channel, color in enumerate(self.COLORS):
            display.setColor(color)
            previous = None
            for timestamp, values in self.samples:
                if values is None:
                    previous = None
                    continue
                x = round(left + (timestamp - start) / self.WINDOW_SECONDS * (right - left))
                y = round((top + bottom) / 2 - values[channel] / scale * (bottom - top) / 2)
                point = (min(right, max(left, x)), min(bottom, max(top, y)))
                if previous is not None:
                    display.drawLine(*previous, *point)
                else:
                    display.drawPixel(*point)
                previous = point
        self._text(f"10 s window | auto +/-{scale:.3g} N | peak {peak:.2f} N", 12, self.height - 16)
