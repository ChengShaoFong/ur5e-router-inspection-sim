"""Offline reachability and sequencing check; Webots is not required."""

import math
import sys
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "controllers" / "dual_arm_router_task"))
from kinematics import TIP_OFFSET, forward_kinematics  # noqa: E402
from plan import ARM1, ARM2, ARM_BASES, joint_trajectory  # noqa: E402


def tool_tip(role, joints):
    frame = forward_kinematics(joints)
    return tuple(
        frame[row][3]
        + sum(frame[row][axis] * TIP_OFFSET[axis] for axis in range(3))
        + ARM_BASES[role][row]
        for row in range(3)
    )


def verify():
    arm1_home = next(c.second for c in ARM1 if c.label == "return home")
    arm2_start = next(c.second for c in ARM2 if c.label == "approach adapter socket")
    arm1_retake = next(c.second for c in ARM1 if c.label == "approach installed adapter")
    assert arm1_home < arm2_start < ARM2[-1].second < arm1_retake

    for role, cues in (("arm1", ARM1), ("arm2", ARM2)):
        points = joint_trajectory(role, cues)
        assert points[0][0] == 0
        if role == "arm2":
            # The 20 cm cleaning head must clear the floor at rest.
            assert tool_tip(role, points[0][1])[2] > 0.25
            assert tool_tip(role, points[-1][1])[2] > 0.25
        assert all(a[0] < b[0] for a, b in zip(points, points[1:]))
        by_time = {round(second, 6): joints for second, joints in points}
        if role == "arm1":
            # Closing the fingers must not start the lift trajectory.
            for second in (8, 9, 10):
                assert math.dist(tool_tip(role, by_time[second]), tool_tip(role, by_time[6])) < 0.001
            assert tool_tip(role, by_time[11])[2] > tool_tip(role, by_time[10])[2] + 0.05
        for cue in cues:
            if cue.tip is not None:
                error = math.dist(tool_tip(role, by_time[round(cue.second, 6)]), cue.tip)
                assert error < 0.006, (role, cue.label, error)

        peak_speed = 0.0
        peak_path_drift = 0.0
        for (start_time, start), (end_time, end) in zip(points, points[1:]):
            duration = end_time - start_time
            speed = max(abs(x - y) for x, y in zip(start, end)) / duration
            peak_speed = max(peak_speed, speed)
            if duration <= 0.251:
                middle = tuple((x + y) / 2 for x, y in zip(start, end))
                expected = tuple((x + y) / 2 for x, y in zip(tool_tip(role, start), tool_tip(role, end)))
                peak_path_drift = max(peak_path_drift, math.dist(tool_tip(role, middle), expected))
        assert peak_speed < 2.5, (role, peak_speed)
        assert peak_path_drift < 0.02, (role, peak_path_drift)
        print(f"{role}: {len(points)} points; peak speed {peak_speed:.2f} rad/s; path drift {peak_path_drift * 1000:.1f} mm")


if __name__ == "__main__":
    verify()
