"""使用簡化 Webots 裝置檢查兩支手臂的控制器啟動流程。"""

import sys
import types
import unittest
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "controllers" / "dual_arm_router_task"))
sys.modules.setdefault("controller", types.SimpleNamespace(Supervisor=object))
from dual_arm_router_task import DualArmTask, corrected_insertion_tip, measured_tool_tip  # noqa: E402
from adapter_vision_tracker import LocalizedAdapter  # noqa: E402
from kinematics import JOINT_NAMES  # noqa: E402
from plan import CueEvent  # noqa: E402
from vision_tracker import LocalizedPort  # noqa: E402


class FakeDevice:
    _tag = 1

    def __init__(self):
        self.position = None
        self.value = 0.0

    def enable(self, period):
        self.period = period

    def setVelocity(self, speed):
        self.speed = speed

    def setPosition(self, position):
        self.position = position

    def getValue(self):
        return self.value

    def attachCamera(self, camera):
        self.camera = camera

    def setAlpha(self, alpha):
        pass

    def fillRectangle(self, *bounds):
        pass

    def setColor(self, color):
        pass

    def drawRectangle(self, *bounds):
        pass


class FakeNode:
    def __init__(self, position=(0, 0, 0)):
        self.position = position

    def getPosition(self):
        return self.position

    def getOrientation(self):
        return (1, 0, 0, 0, 1, 0, 0, 0, 1)

    def getParentNode(self):
        return self

    def setJointPosition(self, angle):
        self.angle = angle


class FakeRobot:
    def __init__(self):
        self.devices = {}
        self.steps = 0

    def getDevice(self, name):
        return self.devices.setdefault(name, FakeDevice())

    def getFromDevice(self, tag):
        return FakeNode()

    def getFromDef(self, name):
        return FakeNode((0.34, 0.33, 0.03) if name == "SERVICE_ADAPTER" else (0.77, 0, 0.28))

    def step(self, period):
        self.steps += 1
        return 0 if self.steps == 1 else -1

    def getTime(self):
        return 0.0


class TaskStartupTest(unittest.TestCase):
    def test_both_roles_start_and_issue_first_joint_command(self):
        for role in ("arm1", "arm2"):
            with self.subTest(role=role):
                robot = FakeRobot()
                task = DualArmTask(robot, role)
                task.run()
                self.assertEqual(task.next_cue, 1)
                self.assertIsNotNone(robot.devices["shoulder_pan_joint"].position)
                if role == "arm1":
                    self.assertIsNotNone(task.vision)
                    self.assertEqual(robot.devices["ROBOTIQ 2F-140 Gripper::left finger joint"].position, 0.0)

    def test_capture_event_replans_insertion_from_vision_and_grip(self):
        robot = FakeRobot()
        task = DualArmTask(robot, "arm1")
        joints = next(values for second, values in task.trajectory if second == 20)
        for name, angle in zip(JOINT_NAMES, joints):
            robot.devices[f"{name}_sensor"].value = angle
        offset = (0.003, -0.002, 0.001)
        tip = measured_tool_tip("arm1", joints)
        task.adapter.position = tuple(a + b for a, b in zip(tip, offset))
        task.vision = types.SimpleNamespace(
            save_capture=lambda directory: None,
            recent=lambda second: LocalizedPort(second, 0.081, 0.299),
        )

        task._capture_and_align(20.0)

        self.assertFalse(task.failed)
        insert = next(cue for cue in task.cues if cue.event == CueEvent.INSERT_ADAPTER)
        expected = corrected_insertion_tip(offset, 0.081, 0.299)
        for actual, target in zip(insert.tip, expected):
            self.assertAlmostEqual(actual, target)

    def test_arm2_uses_moving_adapter_position_and_requires_recent_image(self):
        task = DualArmTask(FakeRobot(), "arm2")
        location = LocalizedAdapter(37.0, 0.608, 0.085, 0.298)
        task.adapter_vision = types.SimpleNamespace(
            recent=lambda second: location,
            save_capture=lambda directory: None,
        )

        task._track_adapter(37.0)

        self.assertFalse(task.failed)
        insert = next(cue for cue in task.cues if cue.event == CueEvent.INSERT_CLEANER)
        self.assertEqual(tuple(round(x, 3) for x in insert.tip), (0.593, 0.085, 0.298))
        task._verify_cleaner_alignment(41.0)
        self.assertFalse(task.failed)
        task.adapter_vision.recent = lambda second: None
        task._verify_cleaner_alignment(41.0)
        self.assertTrue(task.failed)


if __name__ == "__main__":
    unittest.main()
