"""雙手臂 Webots 控制器：依時間表觀測、處理事件並下達關節目標。"""

import os
import sys
from collections import deque
from dataclasses import replace
from pathlib import Path

from controller import Supervisor

from settings import adapter_vision as adapter_config, control as controls, port_vision as port_config
from kinematics import JOINT_NAMES, TIP_OFFSET, forward_kinematics
from plan import (
    ARM1, ARM2, ARM_BASES, CLOSED_ANGLE, INSERTION_TIP, INSTALLED_TIP,
    PLACE_TIP, PORT, POINTS, RUN_ORDER, CueEvent, joint_trajectory,
)
from vision_tracker import AdapterVisionTracker, PortVisionTracker


GRIPPER_MOTORS = (
    "ROBOTIQ 2F-140 Gripper::left finger joint",
    "ROBOTIQ 2F-140 Gripper::right finger joint",
)
GRIPPER_SENSORS = (
    "ROBOTIQ 2F-140 Gripper left finger joint sensor",
    "ROBOTIQ 2F-140 Gripper right finger joint sensor",
)


def event_time(cues, event, point_id=None):
    """取得指定流程事件的秒數；沒有該事件時回傳無限大。"""
    return next((cue.second for cue in cues if cue.event == event
                 and (point_id is None or cue.point_id == point_id)), float("inf"))


def measured_tool_tip(role, joints):
    """由實際關節角計算夾爪工具點的世界座標。"""
    frame = forward_kinematics(joints)
    return tuple(
        frame[row][3]
        + sum(frame[row][axis] * TIP_OFFSET[axis] for axis in range(3))
        + ARM_BASES[role][row]
        for row in range(3)
    )


def corrected_insertion_tip(adapter_offset, port_y, port_z, point_id="port_a"):
    """結合視覺對位與實際夾持偏移，計算插入時的工具點。"""
    return (
        INSERTION_TIP[0] + POINTS[point_id][0] - PORT[0] - adapter_offset[0],
        INSERTION_TIP[1] + port_y - PORT[1] - adapter_offset[1],
        INSERTION_TIP[2] + port_z - PORT[2] - adapter_offset[2],
    )


class DualArmTask:
    """持有一支手臂的裝置與流程狀態；主循環只負責安排每步工作。"""

    def __init__(self, robot, role):
        """載入路徑、感測器，以及對應手臂的夾爪或相機追蹤器。"""
        self.robot = robot
        self.role = role
        self.cues = ARM1 if role == "arm1" else ARM2
        self.trajectory = joint_trajectory(role, self.cues)
        self.pickup_check_time = event_time(self.cues, CueEvent.LIFT_ADAPTER) + controls.PICKUP_CHECK_DELAY
        self.removal_checks = [(event_time(self.cues, CueEvent.REMOVE_ADAPTER, name)
                                + controls.REMOVAL_CHECK_DELAY, name) for name in RUN_ORDER]
        self.next_removal_check = 0
        self.active_point_id = RUN_ORDER[0]
        self.cleaner_insert_time = event_time(ARM2, CueEvent.INSERT_CLEANER, self.active_point_id)
        self.completion_time = max(ARM1[-1].second, ARM2[-1].second) + controls.COMPLETION_DELAY

        trace_path = os.environ.get("DUAL_ARM_TRACE_DIR")
        self.capture_dir = (
            Path(trace_path) if trace_path
            else Path(__file__).resolve().parents[2] / ".dual-arm-runtime"
        )
        if trace_path:
            self.capture_dir.mkdir(parents=True, exist_ok=True)
        self.trace = (
            open(self.capture_dir / f"{role}.log", "w", encoding="utf-8", buffering=1)
            if trace_path else None
        )

        self.adapter = robot.getFromDef("SERVICE_ADAPTER")
        if self.adapter is None:
            raise RuntimeError("SERVICE_ADAPTER is missing from the world")
        self.motors = tuple(robot.getDevice(name) for name in JOINT_NAMES)
        self.sensors = tuple(robot.getDevice(f"{name}_sensor") for name in JOINT_NAMES)
        for sensor in self.sensors:
            sensor.enable(controls.TIME_STEP_MS)
        for motor in self.motors:
            motor.setVelocity(controls.MAX_JOINT_SPEED)

        self.gripper = ()
        self.gripper_sensors = ()
        self.vision = None
        self.adapter_vision = None
        if role == "arm1":
            self._set_initial_arm_pose()
            self.gripper = tuple(robot.getDevice(name) for name in GRIPPER_MOTORS)
            self.gripper_sensors = tuple(robot.getDevice(name) for name in GRIPPER_SENSORS)
            for sensor in self.gripper_sensors:
                sensor.enable(controls.TIME_STEP_MS)
            capture_start = event_time(self.cues, CueEvent.STOP_IN_FRONT) + port_config.SETTLE_SECONDS
            capture_end = event_time(self.cues, CueEvent.CAPTURE_PORT)
            self.vision = PortVisionTracker(robot, capture_start, capture_end)
        else:
            capture_start = event_time(self.cues, CueEvent.ARM1_CLEAR_PORT) + adapter_config.SETTLE_SECONDS
            capture_end = event_time(self.cues, CueEvent.CAPTURE_SOCKET, self.active_point_id)
            self.adapter_vision = AdapterVisionTracker(robot, capture_start, capture_end)

        self.applied_adapter_pose = POINTS[self.active_point_id]
        self.last_adapter_replan_time = float("-inf")

        self.next_cue = 0
        self.next_point = 0
        self.failed = False
        self.abort_target = None
        self.grip_history = deque(maxlen=controls.GRIP_HISTORY_SAMPLES)
        self.grasp_start_position = None
        self.reported_pickup = False
        self.reported_finish = False

    def _set_initial_arm_pose(self):
        """物理模擬開始前將夾爪抬離地板，底座位置不變。"""
        for motor, angle in zip(self.motors, self.trajectory[0][1]):
            robot_joint = self.robot.getFromDevice(motor._tag).getParentNode()
            robot_joint.setJointPosition(angle)

    def run(self):
        """依序執行觀測、事件、狀態檢查、關節命令與完成回報。"""
        try:
            while self.robot.step(controls.TIME_STEP_MS) != -1:
                second = self.robot.getTime()
                if self.vision is not None:
                    self.vision.sample(second)
                if self.adapter_vision is not None:
                    self.adapter_vision.sample(second)
                    self._track_adapter(second)
                if self.gripper_sensors:
                    self.grip_history.append(self._finger_angles())
                self._process_cues(second)
                self._check_milestones(second)
                self._command_joints(second)
                self._report_completion(second)
        finally:
            if self.trace is not None:
                self.trace.close()

    def _process_cues(self, second):
        """依時間處理夾爪命令與流程事件；顯示文字不參與判斷。"""
        while self.next_cue < len(self.cues) and second >= self.cues[self.next_cue].second:
            cue = self.cues[self.next_cue]
            if cue.grip is not None:
                self._command_gripper(cue.grip)
            self._log_cue(second, cue.label)
            self.next_cue += 1
            if cue.event == CueEvent.CHECK_GRASP:
                self._check_grasp()
            elif cue.event == CueEvent.STOP_IN_FRONT:
                self.active_point_id = cue.point_id
                self.vision.set_target(cue.point_id, cue.second + port_config.SETTLE_SECONDS,
                                       event_time(self.cues, CueEvent.CAPTURE_PORT, cue.point_id))
            elif cue.event == CueEvent.ARM1_CLEAR_PORT:
                self.active_point_id = cue.point_id
                self.cleaner_insert_time = event_time(self.cues, CueEvent.INSERT_CLEANER, cue.point_id)
                self.applied_adapter_pose = POINTS[cue.point_id]
                self.adapter_vision.set_target(cue.point_id, cue.second + adapter_config.SETTLE_SECONDS,
                                               event_time(self.cues, CueEvent.CAPTURE_SOCKET, cue.point_id))
            elif cue.event == CueEvent.CAPTURE_PORT and not self.failed:
                self._capture_and_align(second)
            elif cue.event == CueEvent.CAPTURE_SOCKET and not self.failed:
                self.adapter_vision.save_capture(self.capture_dir)
                self._verify_cleaner_alignment(second, max_age=1.0)
            elif cue.event == CueEvent.INSERT_CLEANER and not self.failed:
                self._verify_cleaner_alignment(second)
            if self.failed:
                break

    def _command_gripper(self, angle):
        """設定夾爪開合速度與角度，閉合時記錄物件起始位置。"""
        if angle > 0:
            self.grasp_start_position = tuple(self.adapter.getPosition())
            self.grip_history.clear()
        for motor in self.gripper:
            motor.setVelocity(controls.GRIPPER_CLOSE_SPEED if angle > 0 else controls.GRIPPER_OPEN_SPEED)
            motor.setPosition(angle)
        if self.gripper_sensors:
            self._print(
                f"gripper command={angle:.3f} rad; "
                f"actual fingers={tuple(round(x, 3) for x in self._finger_angles())} rad"
            )

    def _check_grasp(self):
        """用手指停止變動、接觸角度與物件位移檢查抓取結果。"""
        angles = self._finger_angles()
        settled = len(self.grip_history) == self.grip_history.maxlen and all(
            max(values) - min(values) < controls.MAX_FINGER_RANGE for values in zip(*self.grip_history)
        )
        moved = sum(
            (a - b) ** 2 for a, b in zip(self.adapter.getPosition(), self.grasp_start_position)
        ) ** 0.5
        contact = all(
            controls.MIN_CONTACT_ANGLE < angle < CLOSED_ANGLE - controls.CONTACT_ANGLE_MARGIN
            for angle in angles
        )
        self._print(
            f"actual fingers={tuple(round(x, 3) for x in angles)} rad; "
            f"target={CLOSED_ANGLE:.3f}; contact inferred={contact}"
        )
        if not settled or not contact or moved > controls.MAX_GRASP_MOVEMENT:
            self._fail(
                f"grasp FAILED: fingers settled={settled}, contact inferred={contact}, "
                f"adapter moved={moved:.3f} m; inspect x/y/z and contact geometry"
            )
            return
        self._print(f"fingers settled; adapter moved {moved:.3f} m during closing")

    def _capture_and_align(self, second):
        """儲存原始影像，使用穩定定位結果重算插入路徑。"""
        self.vision.save_capture(self.capture_dir)
        port = self.vision.recent(second)
        if port is None:
            self._fail(f"alignment FAILED: stopped hand camera did not localize {self.active_point_id}")
            return

        joints = self._joint_angles()
        tip = measured_tool_tip(self.role, joints)
        offset = tuple(a - t for a, t in zip(self.adapter.getPosition(), tip))
        if max(abs(value) for value in offset) > controls.MAX_ADAPTER_OFFSET:
            self._fail(f"alignment FAILED: adapter slipped {offset}; adjust physical grip before insertion")
            return

        target = corrected_insertion_tip(offset, port.y, port.z, self.active_point_id)
        self.cues = tuple(
            replace(cue, tip=target) if cue.event == CueEvent.INSERT_ADAPTER
            and cue.point_id == self.active_point_id else cue
            for cue in self.cues
        )
        self.trajectory = joint_trajectory(self.role, self.cues)
        self.next_point = 0
        self._print(
            f"camera port y/z={(round(port.y, 4), round(port.z, 4))}; "
            f"adapter offset={tuple(round(x, 4) for x in offset)}; "
            f"corrected insertion tip={tuple(round(x, 4) for x in target)}"
        )

    def _track_adapter(self, second):
        """依手臂二相機的最新插座座標，持續重算接近、插入與退出路徑。"""
        if self.failed or second > self.cleaner_insert_time:
            return
        detection = self.adapter_vision.recent(second)
        if detection is None:
            return
        observed = (detection.x, detection.y, detection.z)
        movement = max(abs(a - b) for a, b in zip(observed, self.applied_adapter_pose))
        if (movement < adapter_config.MIN_REPLAN_SHIFT
                or second - self.last_adapter_replan_time < adapter_config.MIN_REPLAN_INTERVAL):
            return
        delta = tuple(a - b for a, b in zip(observed, POINTS[self.active_point_id]))
        # 拍照點保持不動，視覺修正只作用於拍照後的清潔路徑。
        shifted_events = {
            CueEvent.INSERT_CLEANER, CueEvent.ROTATE_CLEANER, CueEvent.WITHDRAW_CLEANER,
        }
        updated = tuple(
            replace(cue, tip=tuple(value + shift for value, shift in zip(base.tip, delta)))
            if cue.event in shifted_events and cue.point_id == self.active_point_id else cue
            for cue, base in zip(self.cues, ARM2)
        )
        try:
            trajectory = joint_trajectory(self.role, updated)
        except ValueError:
            self._fail(f"cleaning SKIPPED: camera target {observed} is outside reachable poses")
            return
        self.cues = updated
        self.trajectory = trajectory
        self.next_point = 0
        self.applied_adapter_pose = observed
        self.last_adapter_replan_time = second
        self._print(f"adapter camera x/y/z={tuple(round(x, 4) for x in observed)}; cleaning path updated")

    def _verify_cleaner_alignment(self, second, max_age=None):
        """拍照停靠及插入前確認插座定位仍有效。"""
        detection = self.adapter_vision.recent(second)
        if detection is None or (max_age is not None and second - detection.time > max_age):
            self._fail(f"cleaning SKIPPED: arm 2 camera did not localize {self.active_point_id} adapter socket")
        elif max_age is not None:
            self._print(f"{self.active_point_id} socket located at "
                        f"{tuple(round(value, 4) for value in (detection.x, detection.y, detection.z))}")

    def _check_milestones(self, second):
        """在抬起、安裝與拔出後確認物件已到預期區域。"""
        if self.role == "arm1" and not self.reported_pickup and second >= self.pickup_check_time:
            self.reported_pickup = True
            if not self.failed and self.adapter.getPosition()[2] < controls.PICKUP_MIN_HEIGHT:
                self._fail("pickup FAILED: adapter did not rise with the fingers; calibrate pickup x/y/z and grip width")
        if self.role == "arm1" and self.next_removal_check < len(self.removal_checks):
            check_time, point_id = self.removal_checks[self.next_removal_check]
            if second >= check_time:
                self.next_removal_check += 1
                installed_x = INSTALLED_TIP[0] + POINTS[point_id][0] - PORT[0]
                if not self.failed and self.adapter.getPosition()[0] > installed_x - controls.REMOVAL_X_MARGIN:
                    self._fail(f"removal FAILED: adapter stayed near {point_id}; calibrate installed pickup pose or grip width")

    def _command_joints(self, second):
        """沿預先計算的關節路徑線性內插；失敗後固定在當前姿態。"""
        while self.next_point < len(self.trajectory) and second >= self.trajectory[self.next_point][0]:
            self.next_point += 1
        start_time, start_pose = self.trajectory[max(0, self.next_point - 1)]
        if self.next_point < len(self.trajectory):
            end_time, end_pose = self.trajectory[self.next_point]
            fraction = max(0.0, min(1.0, (second - start_time) / (end_time - start_time)))
            target = tuple(a + fraction * (b - a) for a, b in zip(start_pose, end_pose))
        else:
            target = start_pose
        if self.abort_target is not None:
            target = self.abort_target
        for motor, joint in zip(self.motors, target):
            motor.setPosition(joint)

    def _report_completion(self, second):
        """在兩支手臂時程結束後輸出關節與插座位置結果。"""
        if self.reported_finish or second < self.completion_time:
            return
        actual = self._joint_angles()
        joint_error = max(abs(value - desired) for value, desired in zip(actual, self.trajectory[-1][1]))
        adapter_position = self.adapter.getPosition()
        floor_error = (
            sum((a - b) ** 2 for a, b in zip(adapter_position, (PLACE_TIP[0], PLACE_TIP[1], controls.FLOOR_TARGET_Z))) ** 0.5
            if self.role == "arm1" else 0.0
        )
        success = (not self.failed and joint_error < controls.MAX_FINAL_JOINT_ERROR
                   and floor_error < controls.MAX_FINAL_FLOOR_ERROR)
        line = f"{second:.2f} {'complete' if success else 'FAILED'}; final joint error={joint_error:.4f} rad"
        line += f"; joints={tuple(round(x, 3) for x in actual)}"
        line += f"; adapter={tuple(round(x, 4) for x in adapter_position)}; floor error={floor_error:.4f} m"
        self._print(line)
        if self.trace is not None:
            self.trace.write(line + "\n")
        self.reported_finish = True

    def _fail(self, message):
        """停止後續事件，並以感測器實際關節角作為保持位置。"""
        self.failed = True
        self.abort_target = self._joint_angles()
        self.next_cue = len(self.cues)
        self._print(message)

    def _joint_angles(self):
        """讀取六個 UR5e 關節感測器。"""
        return tuple(sensor.getValue() for sensor in self.sensors)

    def _finger_angles(self):
        """讀取兩片夾爪手指的實際角度。"""
        return tuple(sensor.getValue() for sensor in self.gripper_sensors)

    def _log_cue(self, second, label):
        """同時將流程節點輸出至 Webots Console 與可選的追蹤檔。"""
        self._print(f"{second:5.1f}s: {label}")
        if self.trace is not None:
            position = tuple(round(x, 4) for x in self.adapter.getPosition())
            self.trace.write(f"{second:.2f} {label} adapter={position}\n")

    def _print(self, message):
        """統一 Console 訊息前綴，方便分辨兩支手臂。"""
        print(f"[router service {self.role}] {message}")


def main():
    """由 Webots controllerArgs 選擇手臂角色並啟動控制流程。"""
    if len(sys.argv) != 2 or sys.argv[1] not in ("arm1", "arm2"):
        raise RuntimeError('Set controllerArgs to [ "arm1" ] or [ "arm2" ]')
    DualArmTask(Supervisor(), sys.argv[1]).run()


if __name__ == "__main__":
    main()
