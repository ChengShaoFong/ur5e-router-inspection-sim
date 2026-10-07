"""檢查事件識別與插入校正，不依賴 Webots 執行環境。"""

import sys
import types
import unittest
from dataclasses import replace
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "controllers" / "dual_arm_router_task"))
sys.modules.setdefault("controller", types.SimpleNamespace(Supervisor=object))
from dual_arm_router_task import corrected_insertion_tip, event_time  # noqa: E402
from plan import ARM1, CueEvent  # noqa: E402


class TaskEventTest(unittest.TestCase):
    def test_display_label_does_not_control_event_time(self):
        renamed = tuple(replace(cue, label="拍照") if cue.event == CueEvent.CAPTURE_PORT else cue for cue in ARM1)
        self.assertEqual(event_time(renamed, CueEvent.CAPTURE_PORT), 20)
        self.assertEqual(event_time(ARM1, CueEvent.INSERT_ADAPTER), 22)

    def test_insertion_uses_vision_and_grip_offset(self):
        self.assertEqual(
            corrected_insertion_tip((0.002, -0.003, 0.004), 0.081, 0.298),
            (0.598, 0.084, 0.294),
        )


if __name__ == "__main__":
    unittest.main()
