"""雙手臂的時間表與關節軌跡；修改動作位置或秒數時從此檔開始。"""

from dataclasses import dataclass, replace
import math
from enum import Enum

from kinematics import HORIZONTAL, HORIZONTAL_ROLLED, TOP_DOWN, horizontal_roll, interpolate_orientation, solve_pose
from settings import calibration, task


ARM_BASES = {"arm1": (0.0, 0.35, 0.0), "arm2": (0.0, -0.35, 0.0)}
# 手臂 1 的底座仍在地面；初始夾爪抬高以避免碰地。
HOME = (0.0, -1.246, 1.685, 0.208, 0.0, 0.153)
HOMES = {"arm1": HOME, "arm2": (0.0, -1.7, 1.6, 0.0, 0.0, 0.0)}
FLOOR = (0.34, 0.33, 0.030)
PORT = (0.60, 0.08, 0.30)
POINTS = {name.lower(): tuple(map(float, xyz)) for name, xyz in vars(task).items()
          if name != "RUN_ORDER"}
RUN_ORDER = tuple(task.RUN_ORDER)
if not RUN_ORDER or len(set(RUN_ORDER)) != len(RUN_ORDER) or any(name not in POINTS for name in RUN_ORDER):
    raise ValueError("task.ini: run_order must contain unique names from [task]")
if any(len(xyz) != 3 or not all(math.isfinite(value) for value in xyz) for xyz in POINTS.values()):
    raise ValueError("task.ini: each point needs three finite coordinates")
PICKUP_TIP = tuple(calibration.PICKUP_TIP)
INSTALLED_TIP = tuple(calibration.INSTALLED_TIP)
INSERTION_TIP = tuple(calibration.INSERTION_TIP)
PLACE_TIP = tuple(calibration.PLACE_TIP)
CLOSED_ANGLE = max(0.0, min(float(calibration.CLOSED_ANGLE), 0.7))


class CueEvent(str, Enum):
    """控制器使用的事件識別；顯示文字 label 可獨立修改。"""

    HOME = "home"
    CHECK_GRASP = "check_grasp"
    LIFT_ADAPTER = "lift_adapter"
    STOP_IN_FRONT = "stop_in_front"
    CAPTURE_PORT = "capture_port"
    INSERT_ADAPTER = "insert_adapter"
    REMOVE_ADAPTER = "remove_adapter"
    ARM1_CLEAR_PORT = "arm1_clear_port"
    APPROACH_SOCKET = "approach_socket"
    CAPTURE_SOCKET = "capture_socket"
    INSERT_CLEANER = "insert_cleaner"
    ROTATE_CLEANER = "rotate_cleaner"
    WITHDRAW_CLEANER = "withdraw_cleaner"


@dataclass(frozen=True)
class Cue:
    """一個時間點的目標姿態、夾爪指令或流程事件。"""

    second: float
    label: str
    tip: tuple[float, float, float] | None = None
    orientation: tuple | None = None
    grip: float | None = None
    event: CueEvent | None = None
    point_id: str | None = None


_ARM1_TEMPLATE = (
    Cue(0, "home", grip=0.0, event=CueEvent.HOME),
    Cue(2, "approach adapter", (PICKUP_TIP[0], PICKUP_TIP[1], PICKUP_TIP[2] + 0.218), TOP_DOWN),
    Cue(6, "lower fingers around adapter", PICKUP_TIP, TOP_DOWN),
    Cue(7, "close gripper on adapter", grip=CLOSED_ANGLE),
    Cue(10, "hold while fingers close", PICKUP_TIP, TOP_DOWN, event=CueEvent.CHECK_GRASP),
    Cue(13, "lift adapter", (PICKUP_TIP[0], PICKUP_TIP[1], PICKUP_TIP[2] + 0.248), TOP_DOWN, event=CueEvent.LIFT_ADAPTER),
    Cue(15, "carry to router", (0.43, 0.08, 0.40), TOP_DOWN),
    Cue(18, "stop in front of port A", (0.515, 0.08, 0.300), TOP_DOWN, event=CueEvent.STOP_IN_FRONT),
    Cue(20, "capture and locate port A", event=CueEvent.CAPTURE_PORT),
    Cue(22, "insert adapter in port A", INSERTION_TIP, TOP_DOWN, event=CueEvent.INSERT_ADAPTER),
    Cue(24, "release adapter", grip=0.0),
    Cue(27, "clear port", (0.43, 0.08, 0.300), TOP_DOWN),
    Cue(31, "return home", event=CueEvent.HOME),
    Cue(55, "wait for cleaning arm"),
    Cue(58, "approach installed adapter", (0.43, 0.08, 0.300), TOP_DOWN),
    Cue(62, "reach adapter", INSTALLED_TIP, TOP_DOWN),
    Cue(64, "close gripper on installed adapter", grip=CLOSED_ANGLE),
    Cue(66, "wait for port unlock"),
    Cue(70, "remove adapter from router", (0.455, 0.08, 0.300), TOP_DOWN, event=CueEvent.REMOVE_ADAPTER),
    Cue(74, "carry adapter to floor", (0.34, 0.33, 0.24), TOP_DOWN),
    Cue(78, "place adapter on floor", PLACE_TIP, TOP_DOWN),
    Cue(80, "release adapter on floor", grip=0.0),
    Cue(84, "return home", event=CueEvent.HOME),
)

_ARM2_TEMPLATE = (
    Cue(0, "home", event=CueEvent.HOME),
    Cue(34, "wait for arm 1 to clear port", event=CueEvent.ARM1_CLEAR_PORT),
    Cue(36, "approach adapter socket", (0.42, 0.08, 0.30), HORIZONTAL, event=CueEvent.APPROACH_SOCKET),
    Cue(38, "capture and locate adapter socket", (0.42, 0.08, 0.30), HORIZONTAL, event=CueEvent.CAPTURE_SOCKET),
    Cue(41, "insert cleaning head", (0.585, 0.08, 0.30), HORIZONTAL, event=CueEvent.INSERT_CLEANER),
    *(Cue(41 + step * 0.5, f"rotate cleaning head {step * 18} degrees", (0.585, 0.08, 0.30), horizontal_roll(math.radians(step * 18)), event=CueEvent.ROTATE_CLEANER) for step in range(1, 11)),
    Cue(50, "withdraw cleaning head", (0.42, 0.08, 0.30), HORIZONTAL_ROLLED, event=CueEvent.WITHDRAW_CLEANER),
    Cue(54, "return home", event=CueEvent.HOME),
)


def point_tip(tip, point_id):
    """把 Port A 校正後的工具目標平移到指定點位。"""
    return tuple(value + center - origin for value, center, origin in zip(tip, POINTS[point_id], PORT))


def build_schedule(run_order=RUN_ORDER):
    """依點位順序產生雙臂流程；中間點位直接夾著插座移往下一站。"""
    if not run_order or len(set(run_order)) != len(run_order) or any(name not in POINTS for name in run_order):
        raise ValueError("run_order must contain unique configured point names")

    arm1 = [cue for cue in _ARM1_TEMPLATE if cue.second < 15]  # 地面取件只執行一次。
    arm2 = [_ARM2_TEMPLATE[0]]
    delay = 0.0
    for point_id in run_order:
        # B 點位直接旋轉會經過腕部奇異區；保持相機水平，延長清潔頭旋轉時間。
        rotation_scale = 3.0 if point_id == "port_b" else 1.0
        recovery = math.ceil((rotation_scale - 1) * 5)
        for cue in _ARM1_TEMPLATE:
            if not 15 <= cue.second <= 70:
                continue
            arm1.append(replace_cue(cue, delay + (recovery if cue.second >= 55 else 0), point_id))
        for cue in _ARM2_TEMPLATE[1:]:
            if cue.event == CueEvent.ROTATE_CLEANER:
                cue_delay = delay + (cue.second - 41) * (rotation_scale - 1)
            elif cue.event in (CueEvent.WITHDRAW_CLEANER, CueEvent.HOME):
                cue_delay = delay + 5 * (rotation_scale - 1)
            else:
                cue_delay = delay
            arm2.append(replace_cue(cue, cue_delay, point_id))
        last_delay = delay + recovery
        delay += 60 + recovery
    arm1.extend(replace_cue(cue, last_delay) for cue in _ARM1_TEMPLATE if cue.second > 70)
    return tuple(arm1), tuple(arm2)


def replace_cue(cue, delay, point_id=None):
    """複製範本動作，更新時間、點位及工具座標。"""
    return replace(cue, second=cue.second + delay,
                   tip=point_tip(cue.tip, point_id) if cue.tip is not None and point_id else cue.tip,
                   label=cue.label.replace("port A", f"port {point_id.removeprefix('port_').upper()}") if point_id else cue.label,
                   point_id=point_id)


ARM1, ARM2 = build_schedule()

def joint_trajectory(role, cues, sample_seconds=0.25):
    """將笛卡兒目標內插成關節路徑，讓夾爪平順搬運物件。"""
    points = []
    joints = HOMES[role]
    last_tip = None
    last_orientation = None
    last_second = 0.0
    for cue in cues:
        if cue.tip is not None:
            if last_tip is None:
                joints = solve_pose(cue.tip, cue.orientation, ARM_BASES[role], joints)
                points.append((cue.second, joints))
            else:
                duration = cue.second - last_second
                count = max(1, math.ceil(duration / sample_seconds))
                for step in range(1, count + 1):
                    fraction = step / count
                    tip = tuple(a + fraction * (b - a) for a, b in zip(last_tip, cue.tip))
                    orientation = interpolate_orientation(last_orientation, cue.orientation, fraction)
                    joints = solve_pose(tip, orientation, ARM_BASES[role], joints)
                    points.append((last_second + duration * fraction, joints))
            last_tip = cue.tip
            last_orientation = cue.orientation
        elif cue.event == CueEvent.HOME:
            joints = HOMES[role]
            last_tip = None
            last_orientation = None
            points.append((cue.second, joints))
        else:
            points.append((cue.second, joints))
        last_second = cue.second
    return tuple(points)
