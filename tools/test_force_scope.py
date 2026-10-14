"""Verify force history, raw peaks, invalid samples and drawing bounds offline."""

import math
import ctypes
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "controllers" / "dual_arm_router_task"))
from force_scope import ForceScope


class ScopeDevice:
    values = (0.0, 0.0, 0.0)

    def __init__(self):
        self.text = []
        self.frames = 0

    def enable(self, period):
        self.period = period

    def getValues(self):
        return self.values

    def getWidth(self):
        return 560

    def getHeight(self):
        return 250

    def setFont(self, *args):
        pass

    def setColor(self, *args):
        pass

    def fillRectangle(self, *args):
        self.frames += 1

    def drawText(self, text, x, y):
        self.text.append(text)

    def drawLine(self, x1, y1, x2, y2):
        self.drawPixel(x1, y1)
        self.drawPixel(x2, y2)

    def drawPixel(self, x, y):
        assert 0 <= x < 560 and 0 <= y < 250, (x, y)


class ScopeRobot:
    def __init__(self):
        self.device = ScopeDevice()

    def getDevice(self, name):
        return self.device


class ForceScopeTest(unittest.TestCase):
    def test_webots_ctypes_pointer_is_read_with_explicit_bounds(self):
        robot = ScopeRobot()
        storage = (ctypes.c_double * 3)(-3, 4, 12)
        pointer = ctypes.cast(storage, ctypes.POINTER(ctypes.c_double))

        class BoundedPointer:
            def __iter__(self):
                raise AssertionError("Never iterate an unbounded native pointer")

            def __getitem__(self, key):
                if key != slice(None, 3):
                    raise AssertionError("Read exactly three doubles")
                return pointer[key]

        robot.device.values = BoundedPointer()
        scope = ForceScope(robot, "arm1", 32)
        scope.sample(0.032)
        self.assertEqual(scope.samples[-1][1], (-3, 4, 12, 13))

    def test_signed_axes_magnitude_and_short_peak_survive_refresh_throttle(self):
        robot = ScopeRobot()
        scope = ForceScope(robot, "arm1", 32)
        scope.sample(0.032)
        robot.device.values = (-3, 4, 12)
        scope.sample(0.064)
        self.assertEqual(scope.samples[-1][1], (-3, 4, 12, 13))
        self.assertEqual(robot.device.frames, 1)
        robot.device.values = (0, 0, 0)
        scope.sample(0.16)
        self.assertTrue(any("peak 13.00 N" in text for text in robot.device.text))

    def test_rolling_window_and_reset(self):
        scope = ForceScope(ScopeRobot(), "arm2", 32)
        for index in range(400):
            scope.sample(index * 0.032)
        self.assertGreaterEqual(scope.samples[0][0], 399 * 0.032 - 10)
        self.assertLessEqual(len(scope.samples), 314)
        scope.sample(0.032)
        self.assertEqual(len(scope.samples), 1)

    def test_invalid_reading_is_a_gap_and_recovers(self):
        robot = ScopeRobot()
        scope = ForceScope(robot, "arm2", 32)
        robot.device.values = (math.nan, 0, 0)
        scope.sample(0.032)
        self.assertIsNone(scope.samples[-1][1])
        self.assertIn("Fx N/A", robot.device.text)
        robot.device.values = (10000, -20000, 30000)
        scope.sample(0.16)
        self.assertIsNotNone(scope.samples[-1][1])


if __name__ == "__main__":
    unittest.main()
