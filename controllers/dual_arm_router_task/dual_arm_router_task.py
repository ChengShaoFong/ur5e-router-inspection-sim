"""Two UR5e actors for friction gripping and cleaning-head insertion."""

import os
import sys
from pathlib import Path

from controller import Robot, Supervisor

from kinematics import JOINT_NAMES
from plan import ARM1, ARM2, INSTALLED_TIP, PLACE_TIP, joint_trajectory


TIME_STEP = 32
MAX_JOINT_SPEED = 2.5


def main():
    if len(sys.argv) != 2 or sys.argv[1] not in ("arm1", "arm2"):
        raise RuntimeError("Set controllerArgs to [ \"arm1\" ] or [ \"arm2\" ]")
    role = sys.argv[1]
    robot = Supervisor() if role == "arm1" else Robot()
    cues = ARM1 if role == "arm1" else ARM2
    trajectory = joint_trajectory(role, cues)
    trace_dir = os.environ.get("DUAL_ARM_TRACE_DIR")
    trace = open(Path(trace_dir) / f"{role}.log", "w", encoding="utf-8", buffering=1) if trace_dir else None
    motors = [robot.getDevice(name) for name in JOINT_NAMES]
    sensors = [robot.getDevice(f"{name}_sensor") for name in JOINT_NAMES]
    for sensor in sensors:
        sensor.enable(TIME_STEP)
    for motor in motors:
        motor.setVelocity(MAX_JOINT_SPEED)

    gripper = ()
    adapter = None
    if role == "arm1":
        gripper = tuple(robot.getDevice(name) for name in (
            "ROBOTIQ 2F-140 Gripper::left finger joint",
            "ROBOTIQ 2F-140 Gripper::right finger joint",
        ))
        adapter = robot.getFromDef("SERVICE_ADAPTER")
        if adapter is None:
            raise RuntimeError("SERVICE_ADAPTER is missing from the world")

    next_cue = 0
    next_point = 0
    reported_finish = False
    reported_pickup = False
    reported_removal = False
    stage_failed = False
    while robot.step(TIME_STEP) != -1:
        second = robot.getTime()
        while next_cue < len(cues) and second >= cues[next_cue].second:
            cue = cues[next_cue]
            if cue.grip is not None:
                for motor in gripper:
                    motor.setVelocity(0.8)
                    motor.setPosition(cue.grip)
            print(f"[router service {role}] {second:5.1f}s: {cue.label}")
            if trace:
                line = f"{second:.2f} {cue.label}"
                if adapter is not None:
                    line += f" adapter={tuple(round(x, 4) for x in adapter.getPosition())}"
                trace.write(line + "\n")
            next_cue += 1
        if adapter is not None and not reported_pickup and second >= 11:
            reported_pickup = True
            if adapter.getPosition()[2] < 0.10:
                stage_failed = True
                print(f"[router service {role}] pickup FAILED: adapter did not rise with the fingers; calibrate pickup x/y/z and grip width")
        if adapter is not None and not reported_removal and second >= 71:
            reported_removal = True
            if adapter.getPosition()[0] > INSTALLED_TIP[0] - 0.04:
                stage_failed = True
                print(f"[router service {role}] removal FAILED: adapter stayed near port A; calibrate installed pickup pose or grip width")
        while next_point < len(trajectory) and second >= trajectory[next_point][0]:
            next_point += 1
        start_time, start_pose = trajectory[max(0, next_point - 1)]
        if next_point < len(trajectory):
            end_time, end_pose = trajectory[next_point]
            fraction = max(0.0, min(1.0, (second - start_time) / (end_time - start_time)))
            target = tuple(a + fraction * (b - a) for a, b in zip(start_pose, end_pose))
        else:
            target = start_pose
        for motor, joint in zip(motors, target):
            motor.setPosition(joint)
        if not reported_finish and second >= 86:
            actual = tuple(sensor.getValue() for sensor in sensors)
            joint_error = max(abs(value - desired) for value, desired in zip(actual, trajectory[-1][1]))
            adapter_position = adapter.getPosition() if adapter is not None else None
            floor_error = (
                sum((a - b) ** 2 for a, b in zip(adapter_position, (PLACE_TIP[0], PLACE_TIP[1], 0.021))) ** 0.5
                if adapter_position is not None else 0.0
            )
            success = not stage_failed and joint_error < 0.05 and floor_error < 0.05
            line = f"{second:.2f} {'complete' if success else 'FAILED'}; final joint error={joint_error:.4f} rad"
            line += f"; joints={tuple(round(x, 3) for x in actual)}"
            if adapter_position is not None:
                line += f"; adapter={tuple(round(x, 4) for x in adapter_position)}; floor error={floor_error:.4f} m"
            print(f"[router service {role}] {line}")
            if trace:
                trace.write(line + "\n")
            reported_finish = True
if __name__ == "__main__":
    main()
