"""不啟動 Webots，檢查停穩取樣與多張影像定位流程。"""

import sys
import unittest
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "controllers" / "dual_arm_router_task"))
from vision_tracker import PortVisionTracker  # noqa: E402


class FakeCamera:
    _tag = 1

    def enable(self, period):
        self.period = period

    def getImage(self):
        return b"image"


class FakeNode:
    def __init__(self, position):
        self.position = position

    def getPosition(self):
        return self.position

    def getOrientation(self):
        return (1, 0, 0, 0, 1, 0, 0, 0, 1)


class FakeDisplay:
    def __init__(self):
        self.rectangles = []
        self.clear_count = 0

    def attachCamera(self, camera):
        self.camera = camera

    def setAlpha(self, alpha):
        self.alpha = alpha

    def fillRectangle(self, *bounds):
        self.clear_count += 1

    def setColor(self, color):
        self.color = color

    def drawRectangle(self, *bounds):
        self.rectangles.append((self.color, bounds))


class FakeRobot:
    def __init__(self):
        self.camera = FakeCamera()
        self.display = FakeDisplay()

    def getDevice(self, name):
        return self.camera if name == "port_a_camera" else self.display

    def getFromDevice(self, tag):
        return FakeNode((0.45, 0.08, 0.30))

    def getFromDef(self, name):
        return FakeNode((0.77, 0, 0.28))


class VisionTrackerTest(unittest.TestCase):
    def test_stable_frames_allow_recent_result_and_respect_capture_window(self):
        robot = FakeRobot()
        calls = []

        def detector(image, position, orientation, plane_x):
            calls.append((image, plane_x))
            return (0.08 + len(calls) * 0.0001, 0.30, (109, 78, 209, 160), 1.0)

        tracker = PortVisionTracker(robot, 18.5, 20.0, detector=detector)
        tracker.sample(18.0)
        self.assertEqual(calls, [])
        for second in (18.5, 18.65, 18.8, 18.95, 19.1):
            tracker.sample(second)
        self.assertEqual(len(calls), 5)
        self.assertAlmostEqual(calls[0][1], 0.656)
        self.assertAlmostEqual(tracker.recent(19.1).y, 0.0803)
        self.assertIsNone(tracker.recent(19.7))
        self.assertEqual(robot.display.clear_count, 6)  # 初始 AOI + 5 張影像
        self.assertEqual(robot.display.rectangles[-1], (0x00FFFF, (109, 78, 100, 82)))

    def test_unstable_frames_are_not_accepted(self):
        robot = FakeRobot()
        values = iter((0.08, 0.08, 0.085, 0.08, 0.08))
        tracker = PortVisionTracker(
            robot, 18.5, 20.0,
            detector=lambda *_: (next(values), 0.30, (109, 78, 209, 160), 1.0),
        )
        for second in (18.5, 18.65, 18.8, 18.95, 19.1):
            tracker.sample(second)
        self.assertIsNone(tracker.recent(19.1))


if __name__ == "__main__":
    unittest.main()
