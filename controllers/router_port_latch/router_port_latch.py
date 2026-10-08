"""依任務點位順序控制 Router 插孔的鎖扣。"""

import os
import sys
from pathlib import Path

from controller import Robot

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "dual_arm_router_task"))
from plan import ARM1, ARM2, RUN_ORDER, CueEvent


def cue_time(cues, event, point_id):
    """取得指定點位事件的模擬秒數。"""
    return next(cue.second for cue in cues if cue.event == event and cue.point_id == point_id)


robot = Robot()
latches = {name: robot.getDevice(f"{name}_latch") for name in RUN_ORDER}
for latch in latches.values():
    latch.enablePresence(32)

trace_dir = os.environ.get("DUAL_ARM_TRACE_DIR")
trace = open(Path(trace_dir) / "router.log", "w", encoding="utf-8", buffering=1) if trace_dir else None
events = []
for point_id in RUN_ORDER:
    events.extend((
        (cue_time(ARM1, CueEvent.INSERT_ADAPTER, point_id) + 2, point_id, "insert"),
        (cue_time(ARM2, CueEvent.WITHDRAW_CLEANER, point_id) + 4, point_id, "clean"),
        (cue_time(ARM1, CueEvent.REMOVE_ADAPTER, point_id) - 4, point_id, "unlock"),
    ))
events.sort()
next_event = 0

for latch in latches.values():
    latch.lock()

while robot.step(32) != -1:
    second = robot.getTime()
    while next_event < len(events) and second >= events[next_event][0]:
        _, point_id, action = events[next_event]
        latch = latches[point_id]
        if trace:
            trace.write(f"{second:.2f} {point_id} {action} presence={latch.getPresence()} locked={latch.isLocked()}\n")
        if action == "unlock":
            latch.unlock()
        next_event += 1
