"""以橫向開合的夾爪夾取工件，並將工件插入橫向槽。"""

import math
from controller import Supervisor

TIME_STEP = 32
robot = Supervisor()

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
for sensor in sensors:
    sensor.enable(TIME_STEP)

gripper_motors = [
    robot.getDevice("ROBOTIQ 2F-140 Gripper::left finger joint"),
    robot.getDevice("ROBOTIQ 2F-140 Gripper::right finger joint"),
]
gripper_sensors = [
    robot.getDevice("ROBOTIQ 2F-140 Gripper left finger joint sensor"),
    robot.getDevice("ROBOTIQ 2F-140 Gripper right finger joint sensor"),
]
for sensor in gripper_sensors:
    if sensor:
        sensor.enable(TIME_STEP)

plug_node = robot.getFromDef("COMPONENT_PLUG")
slot_node = robot.getFromDef("ROUTER_SLOT_HORIZONTAL")

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
# 夾取前讓夾爪繞接近軸旋轉 90 度；轉向插槽後，夾指沿 Y 軸橫向開合。
TOPDOWN_ROTATION = ((1, 0, 0), (0, -1, 0), (0, 0, -1))
HORIZONTAL_ROTATION = ((0, 0, 1), (0, -1, 0), (1, 0, 0))
# 沿 +X 方向進入橫向槽；先沿 -X 方向退出槽口，再抬升或改變姿態。
SLOT_EXIT_DISTANCE = 0.23
RETURN_CLEARANCE_HEIGHT = 0.50
RETURN_STEP_SIZE = 0.02
GRASP_HEIGHT = 0.030
ROTATION_STEPS = 12
JOINT_LIMITS = [(-2 * math.pi, 2 * math.pi)] * 6
JOINT_LIMITS[2] = (-math.pi, math.pi)


def identity_transform():
    return [[1.0, 0, 0, 0], [0, 1.0, 0, 0], [0, 0, 1.0, 0], [0, 0, 0, 1.0]]


def multiply_transform(a, b):
    return [
        [sum(a[row][k] * b[k][column] for k in range(4)) for column in range(4)]
        for row in range(4)
    ]


def translation_transform(offset):
    transform = identity_transform()
    for axis in range(3):
        transform[axis][3] = offset[axis]
    return transform


def rotation_transform(axis, angle):
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


def forward_kinematics(joints):
    transform = identity_transform()
    for angle, (axis, anchor, endpoint, fixed_rotation) in zip(
        joints, UR5E_LINKS, strict=True
    ):
        transform = multiply_transform(transform, translation_transform(anchor))
        transform = multiply_transform(transform, rotation_transform(axis, angle))
        transform = multiply_transform(
            transform, translation_transform(tuple(-value for value in anchor))
        )
        transform = multiply_transform(transform, translation_transform(endpoint))
        transform = multiply_transform(
            transform, rotation_transform(fixed_rotation[:3], fixed_rotation[3])
        )
    return multiply_transform(transform, translation_transform(TOOL_SLOT_OFFSET))


def orientation_error(target, current):
    relative = [
        [sum(target[row][k] * current[column][k] for k in range(3)) for column in range(3)]
        for row in range(3)
    ]
    return [
        (relative[2][1] - relative[1][2]) / 2,
        (relative[0][2] - relative[2][0]) / 2,
        (relative[1][0] - relative[0][1]) / 2,
    ]


def rotation_step_orientation(step):
    angle = -math.pi / 2 * step / ROTATION_STEPS
    cosine = math.cos(angle)
    sine = math.sin(angle)
    world_y_rotation = (
        (cosine, 0, sine),
        (0, 1, 0),
        (-sine, 0, cosine),
    )
    return tuple(
        tuple(
            sum(
                world_y_rotation[row][axis] * TOPDOWN_ROTATION[axis][column]
                for axis in range(3)
            )
            for column in range(3)
        )
        for row in range(3)
    )


def solve_linear_system(matrix, vector):
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


def solve_pose(grip_point, target_rotation):
    target_position = tuple(
        grip_point[row]
        - sum(target_rotation[row][axis] * GRIP_POINT_OFFSET[axis] for axis in range(3))
        for row in range(3)
    )
    joints = [sensor.getValue() for sensor in sensors]
    epsilon = 1e-5

    for _ in range(120):
        transform = forward_kinematics(joints)
        position_error = [target_position[i] - transform[i][3] for i in range(3)]
        rotation_error = orientation_error(
            target_rotation, [row[:3] for row in transform[:3]]
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
            next_transform = forward_kinematics(perturbed)
            angle_delta = orientation_error(
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
        delta = solve_linear_system(normal, projected_error)
        scale = min(1.0, 0.25 / max(abs(value) for value in delta))
        joints = [
            min(JOINT_LIMITS[i][1], max(JOINT_LIMITS[i][0], joints[i] + delta[i] * scale))
            for i in range(6)
        ]

    raise ValueError(
        f"IK failed for grip point {tuple(round(value, 3) for value in grip_point)}"
    )


def vector_in_world(rotation, vector):
    return [
        sum(rotation[row][axis] * vector[axis] for axis in range(3))
        for row in range(3)
    ]


def vector_in_local(rotation, vector):
    return [
        sum(rotation[row][axis] * vector[row] for row in range(3))
        for axis in range(3)
    ]


def grip_point_position(joints):
    transform = forward_kinematics(joints)
    return [
        transform[row][3]
        + sum(transform[row][axis] * GRIP_POINT_OFFSET[axis] for axis in range(3))
        for row in range(3)
    ]


def pose_for_object(object_center, target_rotation, held_offset_local):
    held_offset_world = vector_in_world(target_rotation, held_offset_local)
    grip_point = [object_center[i] - held_offset_world[i] for i in range(3)]
    return solve_pose(grip_point, target_rotation)


def target_reached(pose, tolerance=0.035):
    return all(
        abs(sensor.getValue() - target) < tolerance
        for sensor, target in zip(sensors, pose, strict=True)
    )


def apply_pose(pose, velocity):
    for motor, target in zip(motors, pose, strict=True):
        motor.setVelocity(velocity)
        motor.setPosition(target)


def control_gripper(position):
    velocity = 0.5 if position > 0 else 0.8
    target = min(0.70, max(0.0, position))
    for motor in gripper_motors:
        if motor:
            motor.setVelocity(velocity)
            motor.setPosition(target)


(
    WAIT_READY,
    MOVE_ABOVE_PLUG,
    DESCEND_TO_PLUG,
    GRIPPER_CLOSING,
    GRIP,
    LIFT,
    ROTATE_HORIZONTAL,
    MOVE_ABOVE_SLOT,
    INSERT_HORIZONTAL,
    OPEN_GRIPPER,
    RETRACT,
    RAISE_CLEAR,
    FINISHED,
    COMPLETE,
    FAILED,
) = range(15)

state = WAIT_READY
watchdog = 0
timer = 0
held_offset_local = [0.0, 0.0, 0.0]
gripper_start_positions = [0.0, 0.0]
gripper_previous_positions = [0.0, 0.0]
gripper_stable_steps = 0
object_target = None
rotation_step = 0
WATCHDOG_LIMIT = 300
ROTATION_WATCHDOG_LIMIT = 600
GRIP_CLOSE_TIMEOUT = 100
HELD_OBJECT_TOLERANCE = 0.3  # 工件相對夾爪預期位置的容許偏差（公尺）
LIFT_CONFIRM_TOLERANCE = 0.025  # 抬升完成時，工件必須緊跟目標高度
SLOT_ALIGNMENT_TOLERANCE = 0.004
RELEASE_STEPS = 50
INSERTION_STEPS = 5
INSERTION_STEP_SIZE = 0.02
INSERTION_Z_OFFSET = 0.003
insertion_step = 0
retract_end_x = None
return_pose = None


def target_reached_or_fail(pose, step_name, tol=0.035, timeout=WATCHDOG_LIMIT):
    global state
    if target_reached(pose, tol):
        return True
    if watchdog > timeout:
        errors = [
            abs(sensor.getValue() - target)
            for sensor, target in zip(sensors, pose, strict=True)
        ]
        worst = max(range(len(errors)), key=errors.__getitem__)
        print(
            f"[錯誤] {step_name} 移動逾時；最大關節差 "
            f"{joint_names[worst]}={errors[worst]:.3f} rad。"
        )
        state = FAILED
    return False


def stop_if_part_slips(stage):
    global state
    joints = [sensor.getValue() for sensor in sensors]
    transform = forward_kinematics(joints)
    grip_position = grip_point_position(joints)
    held_offset_world = vector_in_world(
        [row[:3] for row in transform[:3]], held_offset_local
    )
    expected_position = [
        grip_position[axis] + held_offset_world[axis] for axis in range(3)
    ]
    actual_position = plug_node.getPosition()
    error = math.sqrt(
        sum((actual_position[axis] - expected_position[axis]) ** 2 for axis in range(3))
    )
    if error <= HELD_OBJECT_TOLERANCE:
        return False

    print(f"[錯誤] {stage} 工件滑離夾爪 {error:.3f} m，立即停止手臂。")
    for motor, sensor in zip(motors, sensors, strict=True):
        motor.setVelocity(0.1)
        motor.setPosition(sensor.getValue())
    state = FAILED
    return True


if plug_node is None or slot_node is None:
    print("[錯誤] 找不到 COMPONENT_PLUG 或 ROUTER_SLOT_HORIZONTAL。")
    state = FAILED
if any(motor is None for motor in gripper_motors) or any(
    sensor is None for sensor in gripper_sensors
):
    print("[錯誤] 找不到 Robotiq 左右指馬達或位置感測器。")
    state = FAILED

print(">>> [橫向插槽裝配控制器] 啟動")

while robot.step(TIME_STEP) != -1:
    watchdog += 1
    if timer > 0:
        timer -= 1
        continue

    part_position = plug_node.getPosition() if plug_node else [0.35, 0.35, 0.025]
    slot_position = slot_node.getPosition() if slot_node else [0.65, 0.25, 0.25]

    if state == WAIT_READY:
        control_gripper(0.0)
        pose = solve_pose(
            (part_position[0], part_position[1], 0.14), TOPDOWN_ROTATION
        )
        apply_pose(pose, 0.8)
        watchdog = 0
        state = MOVE_ABOVE_PLUG
        print("1. 移至工件上方")

    elif state == MOVE_ABOVE_PLUG:
        pose = solve_pose(
            (part_position[0], part_position[1], 0.14), TOPDOWN_ROTATION
        )
        if target_reached_or_fail(pose, "移至工件上方"):
            pose_down = solve_pose(
                (part_position[0], part_position[1], GRASP_HEIGHT),
                TOPDOWN_ROTATION,
            )
            apply_pose(pose_down, 0.25)
            watchdog = 0
            state = DESCEND_TO_PLUG
            print("2. 垂直下降至夾取位置")

    elif state == DESCEND_TO_PLUG:
        pose_down = solve_pose(
            (part_position[0], part_position[1], GRASP_HEIGHT),
            TOPDOWN_ROTATION,
        )
        if target_reached_or_fail(pose_down, "下降至夾取高度"):
            gripper_start_positions = [
                sensor.getValue() for sensor in gripper_sensors
            ]
            gripper_previous_positions = gripper_start_positions[:]
            control_gripper(0.70)
            gripper_stable_steps = 0
            watchdog = 0
            state = GRIPPER_CLOSING
            print("3. 開始閉合夾爪")

    elif state == GRIPPER_CLOSING:
        positions = [sensor.getValue() for sensor in gripper_sensors]
        at_target = all(abs(position - 0.70) < 0.02 for position in positions)
        stopped = all(
            abs(position - previous) < 0.001
            for position, previous in zip(
                positions, gripper_previous_positions, strict=True
            )
        )
        gripper_stable_steps = gripper_stable_steps + 1 if stopped else 0
        if at_target:
            print("[錯誤] 夾爪已完全閉合，沒有夾到工件；停止抬升。")
            state = FAILED
        elif gripper_stable_steps >= 8:
            if any(
                abs(position - start) < 0.02
                for position, start in zip(
                    positions, gripper_start_positions, strict=True
                )
            ):
                print("[錯誤] 左右指未能一起夾緊，停止流程。")
                state = FAILED
            else:
                timer = 20
                watchdog = 0
                state = GRIP
                print("3. 夾爪閉合完成")
        elif watchdog > GRIP_CLOSE_TIMEOUT:
            print("[錯誤] 夾爪閉合逾時，停止流程。")
            state = FAILED
        gripper_previous_positions = positions

    elif state == GRIP:
        # 等夾指與工件穩定後，再記錄工件相對夾爪的位置。
        joints = [sensor.getValue() for sensor in sensors]
        transform = forward_kinematics(joints)
        grip_position = grip_point_position(joints)
        world_offset = [part_position[i] - grip_position[i] for i in range(3)]
        held_offset_local = vector_in_local(
            [row[:3] for row in transform[:3]], world_offset
        )
        object_target = (part_position[0], part_position[1], 0.30)
        pose_lift = pose_for_object(
            object_target, TOPDOWN_ROTATION, held_offset_local
        )
        apply_pose(pose_lift, 0.08)
        watchdog = 0
        state = LIFT
        print("4. 垂直提起工件")

    elif state == LIFT:
        if stop_if_part_slips("抬升中"):
            continue
        pose = pose_for_object(object_target, TOPDOWN_ROTATION, held_offset_local)
        if target_reached_or_fail(pose, "垂直提起工件"):
            actual_part_position = plug_node.getPosition()
            lift_error = math.sqrt(
                sum(
                    (actual_part_position[axis] - object_target[axis]) ** 2
                    for axis in range(3)
                )
            )
            if lift_error > LIFT_CONFIRM_TOLERANCE:
                print(
                    f"[錯誤] 工件沒有跟著抬升，與目標差 {lift_error:.3f} m；"
                    "不執行轉向。"
                )
                state = FAILED
            else:
                rotation_step = 1
                intermediate_rotation = rotation_step_orientation(rotation_step)
                pose_horizontal = pose_for_object(
                    object_target, intermediate_rotation, held_offset_local
                )
                apply_pose(pose_horizontal, 0.1)
                watchdog = 0
                state = ROTATE_HORIZONTAL
                print(
                    f"4. 確認工件已抬起至 Z={actual_part_position[2]:.3f} m，"
                    f"開始對稱分段轉向 {rotation_step}/{ROTATION_STEPS}"
                )

    elif state == ROTATE_HORIZONTAL:
        if stop_if_part_slips(
            f"空中轉向第 {rotation_step}/{ROTATION_STEPS} 段"
        ):
            continue
        intermediate_rotation = rotation_step_orientation(rotation_step)
        pose_horizontal = pose_for_object(
            object_target, intermediate_rotation, held_offset_local
        )
        if target_reached_or_fail(
            pose_horizontal,
            f"空中轉向 {rotation_step}/{ROTATION_STEPS}",
            timeout=ROTATION_WATCHDOG_LIMIT,
        ):
            if rotation_step < ROTATION_STEPS:
                rotation_step += 1
                intermediate_rotation = rotation_step_orientation(rotation_step)
                pose_horizontal = pose_for_object(
                    object_target, intermediate_rotation, held_offset_local
                )
                apply_pose(pose_horizontal, 0.1)
                watchdog = 0
                print(f"5. 空中轉向 {rotation_step}/{ROTATION_STEPS}")
            else:
                joints = [sensor.getValue() for sensor in sensors]
                transform = forward_kinematics(joints)
                grip_position = grip_point_position(joints)
                actual_part_position = plug_node.getPosition()
                world_offset = [
                    actual_part_position[axis] - grip_position[axis]
                    for axis in range(3)
                ]
                held_offset_local = vector_in_local(
                    [row[:3] for row in transform[:3]], world_offset
                )
                object_target = (
                    slot_position[0] - 0.15,
                    slot_position[1],
                    slot_position[2],
                )
                pose_approach = pose_for_object(
                    object_target, HORIZONTAL_ROTATION, held_offset_local
                )
                apply_pose(pose_approach, 0.12)
                watchdog = 0
                state = MOVE_ABOVE_SLOT
                print("6. 夾爪保持閉合，對準橫向槽入口")

    elif state == MOVE_ABOVE_SLOT:
        if stop_if_part_slips("移至橫向槽途中"):
            continue
        pose_approach = pose_for_object(
            object_target, HORIZONTAL_ROTATION, held_offset_local
        )
        if target_reached_or_fail(
            pose_approach, "移至橫向槽外側對準", tol=0.01
        ):
            actual_part_position = plug_node.getPosition()
            y_error = slot_position[1] - actual_part_position[1]
            z_error = slot_position[2] - actual_part_position[2]
            if (
                abs(y_error) <= SLOT_ALIGNMENT_TOLERANCE
                and abs(z_error) <= SLOT_ALIGNMENT_TOLERANCE
            ):
                insertion_step = 1
                object_target = (
                    slot_position[0] - 0.15 + INSERTION_STEP_SIZE,
                    slot_position[1],
                    slot_position[2] + INSERTION_Z_OFFSET,
                )
                pose_insert = pose_for_object(
                    object_target, HORIZONTAL_ROTATION, held_offset_local
                )
                apply_pose(pose_insert, 0.06)
                watchdog = 0
                state = INSERT_HORIZONTAL
                print(
                    f"7. 外部對準完成，水平插入第 {insertion_step}/{INSERTION_STEPS} 段；"
                    "工件中心目標 "
                    f"X={object_target[0]:.3f}, Y={object_target[1]:.3f}, "
                    f"Z={object_target[2]:.3f}"
                )
            else:
                print(
                    f"[錯誤] 槽外對位仍超差 Y={y_error:.3f} m, "
                    f"Z={z_error:.3f} m；保持在槽外停止，不做二次上下修正。"
                )
                state = FAILED

    elif state == INSERT_HORIZONTAL:
        if stop_if_part_slips("橫向插入中"):
            continue
        pose_insert = pose_for_object(
            object_target, HORIZONTAL_ROTATION, held_offset_local
        )
        if target_reached_or_fail(pose_insert, "水平插入", tol=0.015):
            if insertion_step < INSERTION_STEPS:
                insertion_step += 1
                object_target = (
                    slot_position[0]
                    - 0.15
                    + INSERTION_STEP_SIZE * insertion_step,
                    slot_position[1],
                    slot_position[2] + INSERTION_Z_OFFSET,
                )
                pose_insert = pose_for_object(
                    object_target, HORIZONTAL_ROTATION, held_offset_local
                )
                apply_pose(pose_insert, 0.06)
                watchdog = 0
                print(f"7. 水平插入第 {insertion_step}/{INSERTION_STEPS} 段")
            else:
                control_gripper(0.0)
                timer = RELEASE_STEPS
                watchdog = 0
                state = OPEN_GRIPPER
                print("8. 槽口外鬆開工件")

    elif state == OPEN_GRIPPER:
        retract_end_x = slot_position[0] - SLOT_EXIT_DISTANCE
        object_target = (
            max(retract_end_x, object_target[0] - RETURN_STEP_SIZE),
            slot_position[1],
            object_target[2],
        )
        pose_retract = pose_for_object(
            object_target, HORIZONTAL_ROTATION, held_offset_local
        )
        apply_pose(pose_retract, 0.3)
        return_pose = pose_retract
        watchdog = 0
        state = RETRACT
        print("9. 沿橫向退出凹槽")

    elif state == RETRACT:
        if target_reached_or_fail(return_pose, "退出橫向槽", tol=0.005):
            if object_target[0] > retract_end_x:
                object_target = (
                    max(retract_end_x, object_target[0] - RETURN_STEP_SIZE),
                    object_target[1], object_target[2]
                )
                pose_retract = pose_for_object(
                    object_target, HORIZONTAL_ROTATION, held_offset_local
                )
                apply_pose(pose_retract, 0.2)
                return_pose = pose_retract
            else:
                placed_position = plug_node.getPosition()
                print(
                    f"[檢查] 退爪後工件位置 X={placed_position[0]:.3f}, "
                    f"Y={placed_position[1]:.3f}, Z={placed_position[2]:.3f} m"
                )
                object_target = (
                    object_target[0], object_target[1],
                    min(RETURN_CLEARANCE_HEIGHT, object_target[2] + RETURN_STEP_SIZE)
                )
                pose_clear = pose_for_object(
                    object_target, HORIZONTAL_ROTATION, held_offset_local
                )
                apply_pose(pose_clear, 0.2)
                return_pose = pose_clear
                state = RAISE_CLEAR
                print("10. 已退出槽口，抬升手臂")
            watchdog = 0

    elif state == RAISE_CLEAR:
        if target_reached_or_fail(return_pose, "raise clear of the slot", tol=0.005):
            if object_target[2] < RETURN_CLEARANCE_HEIGHT:
                object_target = (
                    object_target[0], object_target[1],
                    min(RETURN_CLEARANCE_HEIGHT, object_target[2] + RETURN_STEP_SIZE)
                )
                pose_clear = pose_for_object(
                    object_target, HORIZONTAL_ROTATION, held_offset_local
                )
                apply_pose(pose_clear, 0.2)
                return_pose = pose_clear
            else:
                # 已離開槽口並抬至安全高度，在此姿態結束動作。
                state = FINISHED
            watchdog = 0

    elif state == FINISHED:
        if target_reached_or_fail(return_pose, "抬升至安全高度"):
            print("[完成] 工件已放入插槽，手臂停在安全高度。")
            state = COMPLETE

    elif state in (COMPLETE, FAILED):
        pass
