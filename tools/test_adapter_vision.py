"""手臂二橘色插座辨識與移動座標估計。"""

import sys
import unittest
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "controllers" / "dual_arm_router_task"))
from adapter_vision import locate_adapter  # noqa: E402
from settings import adapter_vision as config  # noqa: E402
CAMERA_WIDTH, CAMERA_HEIGHT = config.CAMERA_WIDTH, config.CAMERA_HEIGHT
from vision_tracker import AdapterVisionTracker  # noqa: E402


def framed_image(left, top, right, bottom):
    """產生橘色中空外框與黃色工具端點的 BGRA 測試影像。"""
    image = bytearray(CAMERA_WIDTH * CAMERA_HEIGHT * 4)

    def rectangle(x1, y1, x2, y2, blue, green, red):
        for y in range(y1, y2):
            for x in range(x1, x2):
                offset = (y * CAMERA_WIDTH + x) * 4
                image[offset:offset + 4] = bytes((blue, green, red, 255))

    orange = (20, 105, 190)
    rectangle(left, top, right, top + 6, *orange)
    rectangle(left, bottom - 6, right, bottom, *orange)
    rectangle(left, top, left + 6, bottom, *orange)
    rectangle(right - 6, top, right, bottom, *orange)
    rectangle(151, 191, 170, 207, 30, 190, 230)  # 黃色清潔頭，不應被當成插座。
    return bytes(image)


class AdapterVisionTest(unittest.TestCase):
    def test_estimates_socket_center_and_depth_from_orange_frame(self):
        image = framed_image(114, 85, 207, 155)
        result = locate_adapter(image, (0.34, 0.08, 0.30), (1, 0, 0, 0, 1, 0, 0, 0, 1))
        self.assertIsNotNone(result)
        x, y, z, box, confidence = result
        self.assertLess(abs(x - 0.60), 0.008)
        self.assertLess(abs(y - 0.08), 0.003)
        self.assertLess(abs(z - 0.30), 0.003)
        self.assertGreater(confidence, 0.2)
        self.assertEqual(box, (115, 85, 205, 153))

    def test_moved_frame_changes_visual_position(self):
        centered = locate_adapter(
            framed_image(114, 85, 207, 155), (0.34, 0.08, 0.30),
            (1, 0, 0, 0, 1, 0, 0, 0, 1),
        )
        moved = locate_adapter(
            framed_image(92, 93, 185, 163), (0.34, 0.08, 0.30),
            (1, 0, 0, 0, 1, 0, 0, 0, 1),
        )
        self.assertIsNotNone(moved)
        self.assertGreater(abs(moved[1] - centered[1]), 0.01)
        self.assertGreater(abs(moved[2] - centered[2]), 0.003)

    def test_tracker_accepts_new_position_after_three_stable_frames(self):
        class Camera:
            _tag = 1

            def enable(self, period):
                pass

            def getImage(self):
                return b"image"

        class Display:
            def attachCamera(self, camera):
                pass

            def setAlpha(self, alpha):
                pass

            def fillRectangle(self, *bounds):
                pass

            def setColor(self, color):
                pass

            def drawRectangle(self, *bounds):
                pass

        class Robot:
            def __init__(self):
                self.camera = Camera()
                self.display = Display()

            def getDevice(self, name):
                return self.camera if name == "adapter_socket_camera" else self.display

            def getFromDevice(self, tag):
                return type("Node", (), {
                    "getPosition": lambda self: (0.34, 0.08, 0.30),
                    "getOrientation": lambda self: (1, 0, 0, 0, 1, 0, 0, 0, 1),
                })()

        positions = iter((0.60, 0.60, 0.60, 0.63, 0.63, 0.63))
        tracker = AdapterVisionTracker(
            Robot(), 34.5, 41.0,
            detector=lambda *_: (next(positions), 0.08, 0.30, (115, 85, 205, 153), 1.0),
        )
        for second in (34.5, 34.7, 34.9):
            tracker.sample(second)
        self.assertAlmostEqual(tracker.recent(34.9).x, 0.60)
        for second in (35.1, 35.3, 35.5):
            tracker.sample(second)
        self.assertAlmostEqual(tracker.recent(35.5).x, 0.63)


if __name__ == "__main__":
    unittest.main()
