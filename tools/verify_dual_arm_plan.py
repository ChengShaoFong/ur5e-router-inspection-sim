"""Offline reachability and sequencing check; Webots is not required."""

import math
import sys
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "controllers" / "dual_arm_router_task"))
from adapter_vision_config import AOI as ADAPTER_AOI, CAMERA_FOV as ADAPTER_FOV  # noqa: E402
from adapter_vision_config import CAMERA_HEIGHT as ADAPTER_HEIGHT, CAMERA_WIDTH as ADAPTER_WIDTH  # noqa: E402
from adapter_vision_config import CAMERA_TOOL_TRANSLATION, CAMERA_TOOL_Y_ROTATION  # noqa: E402
from adapter_vision_config import REAR_FACE_TO_ORIGIN_X  # noqa: E402
from kinematics import TIP_OFFSET, forward_kinematics, rotation  # noqa: E402
from plan import ARM1, ARM2, ARM_BASES, PORT, joint_trajectory  # noqa: E402
from vision_geometry import focal_pixels  # noqa: E402


def tool_tip(role, joints):
    frame = forward_kinematics(joints)
    return tuple(
        frame[row][3]
        + sum(frame[row][axis] * TIP_OFFSET[axis] for axis in range(3))
        + ARM_BASES[role][row]
        for row in range(3)
    )


def adapter_pixel_from_arm2(joints):
    """將標稱插座後端面投影到手臂二相機畫面。"""
    frame = forward_kinematics(joints)
    tool_rotation = [row[:3] for row in frame[:3]]
    camera_rotation = rotation((0, 1, 0), CAMERA_TOOL_Y_ROTATION)
    orientation = [
        [sum(tool_rotation[row][k] * camera_rotation[k][column] for k in range(3)) for column in range(3)]
        for row in range(3)
    ]
    position = [
        frame[row][3] + ARM_BASES["arm2"][row]
        + sum(tool_rotation[row][k] * CAMERA_TOOL_TRANSLATION[k] for k in range(3))
        for row in range(3)
    ]
    rear_face = (PORT[0] - REAR_FACE_TO_ORIGIN_X, PORT[1], PORT[2])
    delta = [rear_face[row] - position[row] for row in range(3)]
    camera_ray = [sum(orientation[row][axis] * delta[row] for row in range(3)) for axis in range(3)]
    focal = focal_pixels(ADAPTER_WIDTH, ADAPTER_FOV)
    return (ADAPTER_WIDTH / 2 - focal * camera_ray[1] / camera_ray[0],
            ADAPTER_HEIGHT / 2 - focal * camera_ray[2] / camera_ray[0])


def verify():
    arm1_home = next(c.second for c in ARM1 if c.label == "return home")
    arm2_start = next(c.second for c in ARM2 if c.label == "approach adapter socket")
    arm1_retake = next(c.second for c in ARM1 if c.label == "approach installed adapter")
    assert arm1_home < arm2_start < ARM2[-1].second < arm1_retake

    for role, cues in (("arm1", ARM1), ("arm2", ARM2)):
        points = joint_trajectory(role, cues)
        assert points[0][0] == 0
        if role == "arm1":
            start, approach = points[0][1], points[1][1]
            for step in range(21):
                fraction = step / 20
                joints = tuple(a + fraction * (b - a) for a, b in zip(start, approach))
                assert tool_tip(role, joints)[2] > 0.12
        if role == "arm2":
            # 20 cm 的清潔頭在待命位置必須離開地板。
            assert tool_tip(role, points[0][1])[2] > 0.25
            assert tool_tip(role, points[-1][1])[2] > 0.25
            for second in (36, 41):
                pixel_x, pixel_y = adapter_pixel_from_arm2(dict(points)[second])
                assert ADAPTER_AOI[0] < pixel_x < ADAPTER_AOI[2]
                assert ADAPTER_AOI[1] < pixel_y < ADAPTER_AOI[3]
        assert all(a[0] < b[0] for a, b in zip(points, points[1:]))
        by_time = {round(second, 6): joints for second, joints in points}
        if role == "arm1":
            front = next(c.second for c in cues if c.label == "stop in front of port A")
            capture = next(c.second for c in cues if c.label == "capture and locate port A")
            insert = next(c.second for c in cues if c.label == "insert adapter in port A")
            assert front < capture < insert
            assert by_time[front] == by_time[capture]
            # Closing the fingers must not start the lift trajectory.
            close = next(c.second for c in cues if c.label == "close gripper on adapter")
            hold = next(c.second for c in cues if c.label == "hold while fingers close")
            lower = next(c.second for c in cues if c.label == "lower fingers around adapter")
            lift = next(c.second for c in cues if c.label == "lift adapter")
            for second in (close, hold):
                assert math.dist(tool_tip(role, by_time[second]), tool_tip(role, by_time[lower])) < 0.001
            midpoint = round((hold + lift) / 2, 6)
            assert tool_tip(role, by_time[midpoint])[2] > tool_tip(role, by_time[hold])[2] + 0.05
            installed_close = next(c.second for c in cues if c.label == "close gripper on installed adapter")
            unlock = next(c.second for c in cues if c.label == "wait for port unlock")
            assert math.dist(tool_tip(role, by_time[installed_close]), tool_tip(role, by_time[unlock])) < 0.001
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
