"""檢查點位設定能重複產生完整流程，並保留跨站直接搬運。"""

import sys
import types
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "controllers" / "dual_arm_router_task"))
sys.modules.setdefault("controller", types.SimpleNamespace(Supervisor=object))
import dual_arm_router_task as task_module  # noqa: E402
from kinematics import JOINT_NAMES  # noqa: E402
from plan import POINTS, CueEvent, build_schedule, joint_trajectory  # noqa: E402
from test_task_startup import FakeRobot  # noqa: E402
from vision_tracker import LocalizedPort  # noqa: E402


class MultiPointPlanTest(unittest.TestCase):
    def test_two_points_transfer_without_floor_place_between_them(self):
        arm1, arm2 = build_schedule(("port_a", "port_b"))
        removals = [cue for cue in arm1 if cue.event == CueEvent.REMOVE_ADAPTER]
        captures = [cue for cue in arm1 if cue.event == CueEvent.CAPTURE_PORT]
        cleaner_inserts = [cue for cue in arm2 if cue.event == CueEvent.INSERT_CLEANER]
        self.assertEqual([(cue.point_id, cue.second) for cue in removals],
                         [("port_a", 70), ("port_b", 130)])
        self.assertEqual([(cue.point_id, cue.second) for cue in captures],
                         [("port_a", 20), ("port_b", 80)])
        self.assertEqual([cue.second for cue in cleaner_inserts], [41, 101])
        between = [cue for cue in arm1 if 70 < cue.second < 82]
        self.assertTrue(any(cue.point_id == "port_b" and cue.tip for cue in between))
        self.assertFalse(any(cue.grip == 0 or cue.event == CueEvent.HOME for cue in between))
        self.assertEqual(arm1[-1].second, 144)
        self.assertEqual(next(cue.tip for cue in arm1 if cue.point_id == "port_b"
                              and cue.event == CueEvent.INSERT_ADAPTER), POINTS["port_b"])
        for role, cues in (("arm1", arm1), ("arm2", arm2)):
            trajectory = joint_trajectory(role, cues)
            self.assertTrue(all(a[0] < b[0] for a, b in zip(trajectory, trajectory[1:])))

    def test_unknown_or_duplicate_point_is_rejected(self):
        with self.assertRaises(ValueError):
            build_schedule(("port_a", "missing"))
        with self.assertRaises(ValueError):
            build_schedule(("port_a", "port_a"))

    def test_second_capture_only_changes_second_insertion(self):
        arm1, arm2 = build_schedule(("port_a", "port_b"))
        with patch.multiple(task_module, ARM1=arm1, ARM2=arm2, RUN_ORDER=("port_a", "port_b")):
            robot = FakeRobot()
            task = task_module.DualArmTask(robot, "arm1")
            task.active_point_id = "port_b"
            joints = dict(task.trajectory)[80]
            for name, angle in zip(JOINT_NAMES, joints):
                robot.devices[f"{name}_sensor"].value = angle
            task.adapter.position = task_module.measured_tool_tip("arm1", joints)
            task.vision = types.SimpleNamespace(
                save_capture=lambda directory: None,
                recent=lambda second: LocalizedPort(second, -0.078, 0.299),
            )
            task._capture_and_align(80)
            self.assertFalse(task.failed)
            inserts = [cue for cue in task.cues if cue.event == CueEvent.INSERT_ADAPTER]
            self.assertEqual(inserts[0].tip, POINTS["port_a"])
            self.assertAlmostEqual(inserts[1].tip[1], -0.078)


if __name__ == "__main__":
    unittest.main()
