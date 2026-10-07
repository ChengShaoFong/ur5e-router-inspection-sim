"""Lock the service adapter to port A, then release it for arm 1 retrieval."""

import os
import sys
from pathlib import Path

from controller import Robot

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "dual_arm_router_task"))
from plan import ARM1, ARM2

cue_times = {cue.label: cue.second for cue in ARM1}
insert_check_time = cue_times["release adapter"]
cleaning_check_time = ARM2[-1].second
release_time = cue_times["wait for port unlock"]

robot = Robot()
latch = robot.getDevice("port_a_latch")
latch.enablePresence(32)
trace_dir = os.environ.get("DUAL_ARM_TRACE_DIR")
trace = open(Path(trace_dir) / "router.log", "w", encoding="utf-8", buffering=1) if trace_dir else None
locked = False
released = False
reported_insert = False
reported_cleaning = False

while robot.step(32) != -1:
    second = robot.getTime()
    if not locked:
        latch.lock()
        locked = True
        if trace:
            trace.write(f"{second:.2f} armed port A latch\n")
    if not reported_insert and second >= insert_check_time:
        if trace:
            trace.write(f"{second:.2f} inserted presence={latch.getPresence()} locked={latch.isLocked()}\n")
        reported_insert = True
    if not reported_cleaning and second >= cleaning_check_time:
        if trace:
            trace.write(f"{second:.2f} after cleaning presence={latch.getPresence()} locked={latch.isLocked()}\n")
        reported_cleaning = True
    if not released and second >= release_time:
        if trace:
            trace.write(f"{second:.2f} port presence={latch.getPresence()}\n")
        latch.unlock()
        released = True
        if trace:
            trace.write(f"{second:.2f} released port A latch\n")
