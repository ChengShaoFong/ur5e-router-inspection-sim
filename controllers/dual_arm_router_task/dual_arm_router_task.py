"""Two UR5e actors for friction gripping and cleaning-head insertion."""

import os
import sys
from collections import deque
from dataclasses import replace
from pathlib import Path

from controller import Supervisor

from kinematics import JOINT_NAMES, TIP_OFFSET, forward_kinematics
from plan import ARM1, ARM2, ARM_BASES, CLOSED_ANGLE, INSERTION_TIP, INSTALLED_TIP, PLACE_TIP, joint_trajectory


TIME_STEP = 32
MAX_JOINT_SPEED = 2.5


def main():
    if len(sys.argv) != 2 or sys.argv[1] not in ("arm1", "arm2"):
        raise RuntimeError("Set controllerArgs to [ \"arm1\" ] or [ \"arm2\" ]")
    role = sys.argv[1]
    robot = Supervisor()
    cues = ARM1 if role == "arm1" else ARM2
    trajectory = joint_trajectory(role, cues)
    cue_times = {cue.label: cue.second for cue in cues}
    pickup_check_time = cue_times.get("lift adapter", float("inf")) + 1
    removal_check_time = cue_times.get("remove adapter from router", float("inf")) + 1
    installation_check_time = ARM2[1].second
    completion_time = max(ARM1[-1].second, ARM2[-1].second) + 2
    trace_dir = os.environ.get("DUAL_ARM_TRACE_DIR")
    trace = open(Path(trace_dir) / f"{role}.log", "w", encoding="utf-8", buffering=1) if trace_dir else None
    motors = [robot.getDevice(name) for name in JOINT_NAMES]
    sensors = [robot.getDevice(f"{name}_sensor") for name in JOINT_NAMES]
    for sensor in sensors:
        sensor.enable(TIME_STEP)
    for motor in motors:
        motor.setVelocity(MAX_JOINT_SPEED)

    gripper = ()
    gripper_sensors = ()
    adapter = robot.getFromDef("SERVICE_ADAPTER")
    if adapter is None:
        raise RuntimeError("SERVICE_ADAPTER is missing from the world")
    if role == "arm1":
        gripper = tuple(robot.getDevice(name) for name in (
            "ROBOTIQ 2F-140 Gripper::left finger joint",
            "ROBOTIQ 2F-140 Gripper::right finger joint",
        ))
        gripper_sensors = tuple(robot.getDevice(name) for name in (
            "ROBOTIQ 2F-140 Gripper left finger joint sensor",
            "ROBOTIQ 2F-140 Gripper right finger joint sensor",
        ))
        for sensor in gripper_sensors:
            sensor.enable(TIME_STEP)

    next_cue = 0
    next_point = 0
    reported_finish = False
    reported_pickup = False
    reported_removal = False
    checked_installation = False
    stage_failed = False
    abort_target = None
    grip_history = deque(maxlen=20)
    grasp_start_position = None
    while robot.step(TIME_STEP) != -1:
        second = robot.getTime()
        if gripper_sensors:
            grip_history.append(tuple(sensor.getValue() for sensor in gripper_sensors))
        while next_cue < len(cues) and second >= cues[next_cue].second:
            cue = cues[next_cue]
            if cue.grip is not None:
                if cue.grip > 0:
                    grasp_start_position = tuple(adapter.getPosition())
                    grip_history.clear()
                for motor in gripper:
                    motor.setVelocity(0.5 if cue.grip > 0 else 0.8)
                    motor.setPosition(cue.grip)
                if gripper_sensors:
                    print(
                        f"[router service {role}] gripper command={cue.grip:.3f} rad; "
                        f"actual fingers={tuple(round(sensor.getValue(), 3) for sensor in gripper_sensors)} rad"
                    )
            print(f"[router service {role}] {second:5.1f}s: {cue.label}")
            if trace:
                line = f"{second:.2f} {cue.label}"
                if adapter is not None:
                    line += f" adapter={tuple(round(x, 4) for x in adapter.getPosition())}"
                trace.write(line + "\n")
            next_cue += 1
            if cue.label.startswith("hold while fingers close"):
                finger_angles = tuple(sensor.getValue() for sensor in gripper_sensors)
                settled = len(grip_history) == grip_history.maxlen and all(
                    max(values) - min(values) < 0.015
                    for values in zip(*grip_history)
                )
                moved = sum((a - b) ** 2 for a, b in zip(adapter.getPosition(), grasp_start_position)) ** 0.5
                physical_contact = all(0.05 < angle < CLOSED_ANGLE - 0.03 for angle in finger_angles)
                print(
                    f"[router service {role}] actual fingers={tuple(round(v, 3) for v in finger_angles)} rad; "
                    f"target={CLOSED_ANGLE:.3f}; contact inferred={physical_contact}"
                )
                if not settled or not physical_contact or moved > 0.012:
                    stage_failed = True
                    abort_target = tuple(sensor.getValue() for sensor in sensors)
                    next_cue = len(cues)
                    print(f"[router service {role}] grasp FAILED: fingers settled={settled}, contact inferred={physical_contact}, adapter moved={moved:.3f} m; inspect x/y/z and contact geometry")
                    break
                print(f"[router service {role}] fingers settled; adapter moved {moved:.3f} m during closing")
            if role == "arm1" and cue.label == "align with port A" and not stage_failed:
                actual_joints = tuple(sensor.getValue() for sensor in sensors)
                frame = forward_kinematics(actual_joints)
                tip = tuple(
                    frame[row][3]
                    + sum(frame[row][axis] * TIP_OFFSET[axis] for axis in range(3))
                    + ARM_BASES[role][row]
                    for row in range(3)
                )
                offset = tuple(a - t for a, t in zip(adapter.getPosition(), tip))
                if max(abs(value) for value in offset) > 0.035:
                    stage_failed = True
                    abort_target = actual_joints
                    next_cue = len(cues)
                    print(f"[router service {role}] alignment FAILED: adapter slipped {offset}; adjust physical grip before insertion")
                    break
                corrected_tip = tuple(goal - delta for goal, delta in zip(INSERTION_TIP, offset))
                cues = tuple(replace(item, tip=corrected_tip) if item.label == "insert adapter in port A" else item for item in cues)
                trajectory = joint_trajectory(role, cues)
                next_point = 0
                print(f"[router service {role}] measured adapter offset={tuple(round(v, 4) for v in offset)}; corrected insertion tip={tuple(round(v, 4) for v in corrected_tip)}")
        if role == "arm1" and not reported_pickup and second >= pickup_check_time:
            reported_pickup = True
            if not stage_failed and adapter.getPosition()[2] < 0.10:
                stage_failed = True
                abort_target = tuple(sensor.getValue() for sensor in sensors)
                next_cue = len(cues)
                print(f"[router service {role}] pickup FAILED: adapter did not rise with the fingers; calibrate pickup x/y/z and grip width")
        if role == "arm2" and not checked_installation and second >= installation_check_time:
            checked_installation = True
            position = adapter.getPosition()
            if position[0] < 0.55 or abs(position[1] - 0.08) > 0.04 or position[2] < 0.24:
                stage_failed = True
                abort_target = tuple(sensor.getValue() for sensor in sensors)
                next_cue = len(cues)
                print(f"[router service {role}] cleaning SKIPPED: adapter is not installed in port A")
        if role == "arm1" and not reported_removal and second >= removal_check_time:
            reported_removal = True
            if not stage_failed and adapter.getPosition()[0] > INSTALLED_TIP[0] - 0.04:
                stage_failed = True
                abort_target = tuple(sensor.getValue() for sensor in sensors)
                next_cue = len(cues)
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
        if abort_target is not None:
            target = abort_target
        for motor, joint in zip(motors, target):
            motor.setPosition(joint)
        if not reported_finish and second >= completion_time:
            actual = tuple(sensor.getValue() for sensor in sensors)
            joint_error = max(abs(value - desired) for value, desired in zip(actual, trajectory[-1][1]))
            adapter_position = adapter.getPosition() if adapter is not None else None
            floor_error = (
                sum((a - b) ** 2 for a, b in zip(adapter_position, (PLACE_TIP[0], PLACE_TIP[1], 0.021))) ** 0.5
                if role == "arm1" else 0.0
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
