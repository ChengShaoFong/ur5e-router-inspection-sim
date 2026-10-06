r"""UR5e + Robotiq 2F-140 精準校準裝配控制器
將 TCP (工具中心點) 實體偏置納入精準補償
"""

import math
from controller import Supervisor

TIME_STEP = 32
robot = Supervisor()

# 啟用攝影機
camera = robot.getDevice("wrist_camera")
if camera:
    camera.enable(TIME_STEP)

# 6 軸關節名稱
joint_names = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

motors = [robot.getDevice(name) for name in joint_names]
sensors = [robot.getDevice(f"{name}_sensor") for name in joint_names]

for s in sensors:
    s.enable(TIME_STEP)

# Robotiq2f140Gripper 的 proto 預設名稱與主動左指關節
gripper_motor = robot.getDevice(
    "ROBOTIQ 2F-140 Gripper::left finger joint"
)
gripper_sensor = robot.getDevice(
    "ROBOTIQ 2F-140 Gripper left finger joint sensor"
)
if gripper_sensor:
    gripper_sensor.enable(TIME_STEP)

# 取得工件與插槽節點
plug_node = robot.getFromDef("COMPONENT_PLUG")
slot_node = robot.getFromDef("ROUTER_SLOT")

# Webots R2025a UR5e joint frames: axis, anchor, endpoint translation, fixed rotation.
UR5E_LINKS = [
    ((0, 0, 1), (0, 0, 0.163), (0, 0, 0.163), (0, 0, 1, 0)),
    ((0, 1, 0), (0, 0.138, 0), (0, 0.138, 0), (0, 1, 0, math.pi / 2)),
    ((0, 1, 0), (0, -0.131, 0.425), (0, -0.131, 0.425), (0, 1, 0, 0)),
    ((0, 1, 0), (0, 0, 0.392), (0, 0, 0.392), (0, 1, 0, math.pi / 2)),
    ((0, 0, 1), (0, 0.127, 0), (0, 0.127, 0), (0, 0, 1, 0)),
    ((0, 1, 0), (0, 0, 0.1), (0, 0, 0.1), (0, 1, 0, 0)),
]
TOOL_SLOT_OFFSET = (0, 0.1, 0)
GRIP_POINT_OFFSET = (0, 0, 0.20)
TOPDOWN_ROTATION = ((0, 1, 0), (1, 0, 0), (0, 0, -1))
JOINT_LIMITS = [(-2 * math.pi, 2 * math.pi)] * 6
JOINT_LIMITS[2] = (-math.pi, math.pi)


def _identity_transform():
    return [[1.0, 0, 0, 0], [0, 1.0, 0, 0], [0, 0, 1.0, 0], [0, 0, 0, 1.0]]


def _multiply_transform(a, b):
    return [
        [sum(a[row][k] * b[k][column] for k in range(4)) for column in range(4)]
        for row in range(4)
    ]


def _translation_transform(offset):
    transform = _identity_transform()
    for axis in range(3):
        transform[axis][3] = offset[axis]
    return transform


def _rotation_transform(axis, angle):
    length = math.sqrt(sum(value * value for value in axis))
    x, y, z = (value / length for value in axis)
    cosine = math.cos(angle)
    sine = math.sin(angle)
    one_minus_cosine = 1 - cosine
    return [
        [one_minus_cosine * x * x + cosine, one_minus_cosine * x * y - sine * z, one_minus_cosine * x * z + sine * y, 0],
        [one_minus_cosine * x * y + sine * z, one_minus_cosine * y * y + cosine, one_minus_cosine * y * z - sine * x, 0],
        [one_minus_cosine * x * z - sine * y, one_minus_cosine * y * z + sine * x, one_minus_cosine * z * z + cosine, 0],
        [0, 0, 0, 1],
    ]


def _forward_kinematics(joints):
    transform = _identity_transform()
    for angle, (axis, anchor, endpoint, fixed_rotation) in zip(
        joints, UR5E_LINKS, strict=True
    ):
        transform = _multiply_transform(transform, _translation_transform(anchor))
        transform = _multiply_transform(transform, _rotation_transform(axis, angle))
        transform = _multiply_transform(
            transform, _translation_transform(tuple(-value for value in anchor))
        )
        transform = _multiply_transform(transform, _translation_transform(endpoint))
        transform = _multiply_transform(
            transform, _rotation_transform(fixed_rotation[:3], fixed_rotation[3])
        )
    return _multiply_transform(transform, _translation_transform(TOOL_SLOT_OFFSET))


def _grip_point_position(joints):
    transform = _forward_kinematics(joints)
    return [
        transform[row][3]
        + sum(transform[row][axis] * GRIP_POINT_OFFSET[axis] for axis in range(3))
        for row in range(3)
    ]


def _slot_grip_target(slot_position, height, held_offset):
    return (
        slot_position[0] - held_offset[0],
        slot_position[1] - held_offset[1],
        slot_position[2] + height - held_offset[2],
    )


def _orientation_error(target, current):
    relative = [
        [sum(target[row][k] * current[column][k] for k in range(3)) for column in range(3)]
        for row in range(3)
    ]
    return [
        (relative[2][1] - relative[1][2]) / 2,
        (relative[0][2] - relative[2][0]) / 2,
        (relative[1][0] - relative[0][1]) / 2,
    ]


def _solve_linear_system(matrix, vector):
    size = len(vector)
    augmented = [matrix[row][:] + [vector[row]] for row in range(size)]
    for column in range(size):
        pivot = max(range(column, size), key=lambda row: abs(augmented[row][column]))
        augmented[column], augmented[pivot] = augmented[pivot], augmented[column]
        divisor = augmented[column][column]
        if abs(divisor) < 1e-12:
            raise ValueError("UR5e IK matrix is singular")
        augmented[column] = [value / divisor for value in augmented[column]]
        for row in range(size):
            if row != column:
                factor = augmented[row][column]
                augmented[row] = [
                    augmented[row][index] - factor * augmented[column][index]
                    for index in range(size + 1)
                ]
    return [augmented[row][-1] for row in range(size)]


def calculate_tcp_topdown_pose(tx, ty, tz_grip):
    """Solve Webots UR5e joint angles for the desired finger-pad grip point."""
    target_position = tuple(
        coordinate
        - sum(
            TOPDOWN_ROTATION[row][axis] * GRIP_POINT_OFFSET[axis]
            for axis in range(3)
        )
        for row, coordinate in enumerate((tx, ty, tz_grip))
    )
    joints = [sensor.getValue() for sensor in sensors]
    epsilon = 1e-5

    for _ in range(100):
        transform = _forward_kinematics(joints)
        position_error = [target_position[i] - transform[i][3] for i in range(3)]
        rotation_error = _orientation_error(
            TOPDOWN_ROTATION, [row[:3] for row in transform[:3]]
        )
        error = position_error + rotation_error
        if (
            math.sqrt(sum(value * value for value in position_error)) < 0.003
            and math.sqrt(sum(value * value for value in rotation_error)) < 0.03
        ):
            return joints

        jacobian = [[0.0] * 6 for _ in range(6)]
        for column in range(6):
            perturbed = joints[:]
            perturbed[column] += epsilon
            next_transform = _forward_kinematics(perturbed)
            angle_delta = _orientation_error(
                [row[:3] for row in next_transform[:3]],
                [row[:3] for row in transform[:3]],
            )
            for row in range(3):
                jacobian[row][column] = (
                    next_transform[row][3] - transform[row][3]
                ) / epsilon
                jacobian[row + 3][column] = angle_delta[row] / epsilon

        normal = [
            [
                sum(jacobian[k][i] * jacobian[k][j] for k in range(6))
                + (0.01 if i == j else 0)
                for j in range(6)
            ]
            for i in range(6)
        ]
        projected_error = [
            sum(jacobian[k][i] * error[k] for k in range(6)) for i in range(6)
        ]
        delta = _solve_linear_system(normal, projected_error)
        scale = min(1.0, 0.25 / max(abs(value) for value in delta))
        joints = [
            min(JOINT_LIMITS[i][1], max(JOINT_LIMITS[i][0], joints[i] + delta[i] * scale))
            for i in range(6)
        ]

    raise ValueError(
        f"UR5e IK did not converge for grip point ({tx:.3f}, {ty:.3f}, {tz_grip:.3f})"
    )


# 狀態定義
(
    WAIT_READY,
    MOVE_ABOVE_PLUG,
    DESCEND_TO_PLUG,
    GRIPPER_CLOSING,
    GRIP,
    LIFT,
    MOVE_ABOVE_SLOT,
    INSERT_INTO_SLOT,
    OPEN_GRIPPER,
    RETRACT,
    RETURN_HOME,
    FINISHED,
    FAILED,
) = range(13)

state = WAIT_READY
timer = 0
watchdog = 0
home_pose = None
held_object_offset = [0.0, 0.0, 0.0]
WATCHDOG_LIMIT = 300
GRIP_CLOSE_TARGET = 0.70
GRIP_CLOSE_TIMEOUT = 100
GRIP_SETTLE_STEPS = 20
SLOT_RELEASE_HEIGHT = 0.1
gripper_start_position = 0.0
gripper_previous_position = 0.0
gripper_stable_steps = 0


def apply_pose(pose, vel=1.0):
    for m, p in zip(motors, pose, strict=False):
        m.setVelocity(vel)
        m.setPosition(p)


def target_reached(pose, tol=0.04):
    return all(
        abs(s.getValue() - p) < tol for s, p in zip(sensors, pose, strict=False)
    )


def target_reached_or_fail(pose, step_name):
    global state
    if target_reached(pose):
        return True
    if watchdog > WATCHDOG_LIMIT:
        errors = [
            abs(sensor.getValue() - target)
            for sensor, target in zip(sensors, pose, strict=False)
        ]
        worst_joint = max(range(len(errors)), key=errors.__getitem__)
        print(
            f"[錯誤] {step_name} 移動逾時；最大關節差 "
            f"{joint_names[worst_joint]}={errors[worst_joint]:.3f} rad。"
        )
        state = FAILED
    return False


def control_gripper(position):
    """2F-140 行程: 0.0 為全開，0.70 為緊密夾合"""
    if gripper_motor:
        gripper_motor.setVelocity(0.5 if position > 0 else 0.8)
        gripper_motor.setPosition(min(0.70, max(0.0, position)))


print(">>> [TCP 精準對齊裝配控制器] 啟動中...")

while robot.step(TIME_STEP) != -1:
    watchdog += 1
    if timer > 0:
        timer -= 1
        continue

    # 動態取得工件與插槽真實中心座標
    p_xyz = plug_node.getPosition() if plug_node else [0.35, 0.35, 0.025]
    s_xyz = slot_node.getPosition() if slot_node else [0.45, 0.0, 0.0]

    # 目標工件中心: p_xyz[0], p_xyz[1]
    # 工件高度 0.05m，工件頂面在 Z=0.05m，腰部夾持點在 Z=0.025m

    if state == WAIT_READY:
        if home_pose is None:
            home_pose = [sensor.getValue() for sensor in sensors]
        control_gripper(0.0)  # 全開
        # 移動至工件正上方 14 公分處
        pose = calculate_tcp_topdown_pose(p_xyz[0], p_xyz[1], tz_grip=0.14)
        apply_pose(pose, vel=1.0)
        watchdog = 0
        state = MOVE_ABOVE_PLUG
        print(f"1. 移動至工件正上方: X={p_xyz[0]:.3f}, Y={p_xyz[1]:.3f}")

    elif state == MOVE_ABOVE_PLUG:
        pose = calculate_tcp_topdown_pose(p_xyz[0], p_xyz[1], tz_grip=0.14)
        if target_reached_or_fail(pose, "移至工件上方"):
            # 指尖垂直下探至工件腰部 (Z=0.025m)
            pose_down = calculate_tcp_topdown_pose(
                p_xyz[0], p_xyz[1], tz_grip=0.035
            )
            apply_pose(pose_down, vel=0.3)
            watchdog = 0
            state = DESCEND_TO_PLUG
            print("2. 夾取中心垂直下降至工件中心 ")

    elif state == DESCEND_TO_PLUG:
        pose_down = calculate_tcp_topdown_pose(p_xyz[0], p_xyz[1], tz_grip=0.095)
        if target_reached_or_fail(pose_down, "下降至夾取高度"):
            if gripper_motor is None or gripper_sensor is None:
                print("[錯誤] 找不到夾爪馬達或位置感測器，停止流程。")
                state = FAILED
            else:
                control_gripper(GRIP_CLOSE_TARGET)
                gripper_start_position = gripper_sensor.getValue()
                gripper_previous_position = gripper_start_position
                gripper_stable_steps = 0
                watchdog = 0
                state = GRIPPER_CLOSING
                print("3. 開始閉合夾爪，等待夾持確認")

    elif state == GRIPPER_CLOSING:
        gripper_position = gripper_sensor.getValue()
        if abs(gripper_position - GRIP_CLOSE_TARGET) < 0.02:
            print("3. 夾爪到達閉合位置")
            timer = GRIP_SETTLE_STEPS
            watchdog = 0
            state = GRIP
        else:
            if abs(gripper_position - gripper_previous_position) < 0.001:
                gripper_stable_steps += 1
            else:
                gripper_stable_steps = 0

            if gripper_stable_steps >= 8:
                if abs(gripper_position - gripper_start_position) >= 0.02:
                    print("3. 夾爪停止移動，視為接觸工件")
                    timer = GRIP_SETTLE_STEPS
                    watchdog = 0
                    state = GRIP
                else:
                    print("[錯誤] 夾爪位置未變，未能確認閉合，停止流程。")
                    state = FAILED
            elif watchdog > GRIP_CLOSE_TIMEOUT:
                print("[錯誤] 夾爪閉合逾時，停止流程。")
                state = FAILED

        gripper_previous_position = gripper_position

    elif state == GRIP:
        # 提起至空中
        pose_lift = calculate_tcp_topdown_pose(p_xyz[0], p_xyz[1], tz_grip=0.18)
        apply_pose(pose_lift, vel=0.2)
        watchdog = 0
        state = LIFT
        print("4. 提升離開地面")

    elif state == LIFT:
        pose_lift = calculate_tcp_topdown_pose(p_xyz[0], p_xyz[1], tz_grip=0.18)
        if target_reached_or_fail(pose_lift, "提起工件"):
            grip_position = _grip_point_position(
                [sensor.getValue() for sensor in sensors]
            )
            held_object_offset = [
                p_xyz[axis] - grip_position[axis] for axis in range(3)
            ]
            print(
                "[校正] 工件相對夾取點偏移 "
                f"X={held_object_offset[0]:.3f}, "
                f"Y={held_object_offset[1]:.3f}, "
                f"Z={held_object_offset[2]:.3f} m"
            )
            # 移至插槽中心上方安全高度 (Z=0.26m)
            slot_target = _slot_grip_target(
                s_xyz, 0.26, held_object_offset
            )
            pose_slot_up = calculate_tcp_topdown_pose(
                *slot_target
            )
            apply_pose(pose_slot_up, vel=0.45)
            watchdog = 0
            state = MOVE_ABOVE_SLOT
            print(f"5. 移至插槽正上方對準: X={s_xyz[0]:.3f}, Y={s_xyz[1]:.3f}")

    elif state == MOVE_ABOVE_SLOT:
        slot_target = _slot_grip_target(s_xyz, 0.26, held_object_offset)
        pose_slot_up = calculate_tcp_topdown_pose(
            *slot_target
        )
        if target_reached_or_fail(pose_slot_up, "移至插槽上方"):
            # 在槽口上方釋放，夾爪不進入插槽
            slot_target = _slot_grip_target(
                s_xyz, SLOT_RELEASE_HEIGHT, held_object_offset
            )
            pose_insert = calculate_tcp_topdown_pose(
                *slot_target
            )
            apply_pose(pose_insert, vel=0.2)
            watchdog = 0
            state = INSERT_INTO_SLOT
            print("6. 下降至槽口外釋放高度")

    elif state == INSERT_INTO_SLOT:
        slot_target = _slot_grip_target(
            s_xyz, SLOT_RELEASE_HEIGHT, held_object_offset
        )
        pose_insert = calculate_tcp_topdown_pose(
            *slot_target
        )
        if target_reached_or_fail(pose_insert, "插入工件"):
            control_gripper(0.0)  # 鬆開工件
            timer = 40
            watchdog = 0
            state = OPEN_GRIPPER
            print("7. 在槽口外釋放工件")

    elif state == OPEN_GRIPPER:
        # 手臂退回安全高度
        slot_target = _slot_grip_target(s_xyz, 0.1, held_object_offset)
        pose_slot_up = calculate_tcp_topdown_pose(
            *slot_target
        )
        apply_pose(pose_slot_up, vel=0.7)
        watchdog = 0
        state = RETRACT
        print("8. 垂直退回安全高度")

    elif state == RETRACT:
        slot_target = _slot_grip_target(s_xyz, 0.1, held_object_offset)
        pose_slot_up = calculate_tcp_topdown_pose(
            *slot_target
        )
        if target_reached_or_fail(pose_slot_up, "手臂退回"):
            apply_pose(home_pose, vel=0.7)
            watchdog = 0
            state = RETURN_HOME
            print("9. 返回控制器啟動時的關節位置")

    elif state == RETURN_HOME:
        if target_reached_or_fail(home_pose, "返回初始位置"):
            print("[完成] 插槽裝配任務完成，手臂已返回初始位置。")
            state = FINISHED

    elif state in (FINISHED, FAILED):
        pass