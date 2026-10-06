"""夾取連著可彎曲線材的 RJ45 插頭，並插入獨立場景的路由器孔。"""

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
grasp_latch = robot.getDevice("gripper_grasp_latch")
camera = robot.getDevice("pickup_camera")
if camera:
    camera.enable(TIME_STEP)
    camera.recognitionEnable(TIME_STEP)
if grasp_latch:
    grasp_latch.enablePresence(TIME_STEP)
for sensor in gripper_sensors:
    if sensor:
        sensor.enable(TIME_STEP)

plug_node = robot.getFromDef("RJ45_PLUG")
slot_node = robot.getFromDef("ROUTER_PORT")
gripper_connector_node = robot.getFromDef("GRIPPER_GRASP_CONNECTOR")
plug_connector_node = robot.getFromDef("PLUG_GRASP_CONNECTOR")
camera_node = robot.getFromDef("PICKUP_CAMERA")
cable_tail_node = robot.getFromDef("CABLE_SEGMENT_18")

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
# 夾爪由上方夾住插頭後側，搬運時保持插頭尖端朝 +X。
TOPDOWN_ROTATION = ((1, 0, 0), (0, -1, 0), (0, 0, -1))
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


# 與方塊實驗完全分離的操作流程與容許誤差。
APPROACH_X = 0.42
INSERT_X = 0.52
PORT_Z = 0.30
PORT_ALIGNMENT_TOLERANCE_Y = 0.012
PORT_ALIGNMENT_TOLERANCE_Z = 0.008
GRASP_REAR_OFFSET = -0.015
LIFT_Z = 0.30
SAFE_Z = 0.48
POSE_TOLERANCE = 0.04
OBJECT_TOLERANCE = 0.025
RELEASE_TARGET = 0.30
# 線材拉力使夾持中的插頭略偏離孔中心；釋放後仍會檢查是否留在孔內。
LATCH_ALIGNMENT_TOLERANCE = 0.015
WATCHDOG_LIMIT = 1200
# 相機逐區巡看地面；每個位置都只是觀察點，抓取點由辨識結果決定。
OBSERVATION_POINTS = (
    (0.35, 0.35, 0.38),
    (0.35, -0.35, 0.38),
    (0.55, 0.0, 0.38),
    (0.15, 0.0, 0.38),
    (0.45, 0.45, 0.38),
    (0.45, -0.45, 0.38),
)
FLOOR_GRASP_Z = 0.025

(
    MOVE_ABOVE,
    DESCEND,
    CLOSE,
    SETTLE,
    LATCH_SETTLE,
    LIFT,
    MOVE_TO_PORT,
    INSERT,
    OPEN,
    RETRACT,
    RAISE,
    COMPLETE,
    FAILED,
    OBSERVE,
    ROTATE_FOR_PORT,
) = range(15)

state = MOVE_ABOVE
watchdog = 0
settle_steps = 0
gripper_stable_steps = 0
previous_gripper = [0.0, 0.0]
held_offset_local = [0.0, 0.0, 0.0]
target_center = None
pose_target = None
step_x = APPROACH_X
alignment_attempts = 0
insertion_corrections = 0
alignment_bias_y = 0.0
alignment_bias_z = 0.0
detected_plug = None
grasp_rotation = TOPDOWN_ROTATION
transport_rotation = TOPDOWN_ROTATION
vision_wait_steps = 0
scan_index = 0
lift_z = 0.05
rotation_step = 0
rotation_center = None


def see_plug():
    """從相機辨識的標記位置與方向計算插頭中心及平面角度。"""
    if camera is None or camera_node is None:
        return None
    objects = [
        obj for obj in camera.getRecognitionObjects()
        if obj.getModel() == "rj45_grasp_target"
    ]
    if len(objects) != 1:
        return None
    local = objects[0].getPosition()
    rotation = camera_node.getOrientation()
    origin = camera_node.getPosition()
    world = [
        origin[i] + sum(rotation[3 * i + j] * local[j] for j in range(3))
        for i in range(3)
    ]
    axis_angle = objects[0].getOrientation()
    marker_rotation = rotation_transform(axis_angle[:3], axis_angle[3])
    object_x = [
        sum(rotation[3 * i + j] * marker_rotation[j][0] for j in range(3))
        for i in range(3)
    ]
    object_z = [
        sum(rotation[3 * i + j] * marker_rotation[j][2] for j in range(3))
        for i in range(3)
    ]
    # 橘色標記在插頭局部 Z 軸上方 14 mm。
    center = [world[i] - 0.014 * object_z[i] for i in range(3)]
    return center, math.atan2(object_x[1], object_x[0])


def floor_grasp_rotation(yaw):
    """讓夾爪從斜上方接近，夾指沿插頭寬度方向閉合。"""
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    diagonal = math.sqrt(0.5)
    return (
        (diagonal * cosine, sine, diagonal * cosine),
        (diagonal * sine, -cosine, diagonal * sine),
        (diagonal, 0.0, -diagonal),
    )


def distance(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def fail(message):
    global state
    print(f"[RJ45 錯誤] {message}")
    if pose_target is not None:
        differences = [
            abs(sensor.getValue() - target)
            for sensor, target in zip(sensors, pose_target, strict=True)
        ]
        worst = max(range(6), key=differences.__getitem__)
        print(
            f"[診斷] 最大關節誤差 {joint_names[worst]}={differences[worst]:.3f} rad；"
            f"插頭位置={tuple(round(v, 3) for v in plug_node.getPosition())}；"
            f"夾指角度={tuple(round(s.getValue(), 3) for s in gripper_sensors)}"
        )
        if cable_tail_node:
            print(
                f"[診斷] 插頭目標={target_center}；"
                f"線尾位置={tuple(round(v, 3) for v in cable_tail_node.getPosition())}"
            )
        if gripper_connector_node and plug_connector_node:
            print(
                f"[診斷] 夾爪連接點={tuple(round(v, 3) for v in gripper_connector_node.getPosition())}；"
                f"插頭連接點={tuple(round(v, 3) for v in plug_connector_node.getPosition())}"
            )
    for motor, sensor in zip(motors, sensors, strict=True):
        motor.setPosition(sensor.getValue())
    state = FAILED


def move_object(center, speed, rotation=None):
    global target_center, pose_target, watchdog
    if rotation is None:
        rotation = transport_rotation
    target_center = center
    pose_target = pose_for_object(center, rotation, held_offset_local)
    apply_pose(pose_target, speed)
    watchdog = 0


def reached(label, tolerance=POSE_TOLERANCE):
    if target_reached(pose_target, tolerance):
        return True
    if watchdog > WATCHDOG_LIMIT:
        fail(f"{label} 移動逾時")
    return False


def held(label):
    if gripper_connector_node and plug_connector_node:
        gap = distance(
            gripper_connector_node.getPosition(),
            plug_connector_node.getPosition(),
        )
        if gap > 0.025:
            fail(f"{label} 夾持連接點分離 {gap:.3f} m")
            return False
        return True
    joints = [sensor.getValue() for sensor in sensors]
    transform = forward_kinematics(joints)
    grip = grip_point_position(joints)
    offset = vector_in_world([row[:3] for row in transform[:3]], held_offset_local)
    expected = [grip[i] + offset[i] for i in range(3)]
    actual = plug_node.getPosition()
    error = distance(expected, actual)
    if error > OBJECT_TOLERANCE:
        if gripper_connector_node and plug_connector_node:
            print(
                f"[診斷] 夾爪連接點={tuple(round(v, 3) for v in gripper_connector_node.getPosition())}，"
                f"插頭連接點={tuple(round(v, 3) for v in plug_connector_node.getPosition())}，"
                f"連接感測={grasp_latch.getPresence()}"
            )
        fail(
            f"{label} 插頭偏離夾爪預期位置 {error:.3f} m；"
            f"預期={tuple(round(v, 3) for v in expected)}，"
            f"實際={tuple(round(v, 3) for v in actual)}"
        )
        return False
    return True


if plug_node is None or slot_node is None:
    fail("找不到 RJ45_PLUG 或 ROUTER_PORT")
if any(device is None for device in gripper_motors + gripper_sensors):
    fail("找不到左右夾指馬達或感測器")
if grasp_latch is None:
    fail("找不到夾持護套的連接點")
if camera is None or camera_node is None:
    fail("找不到夾爪視覺相機")
print(">>> [RJ45 柔性線材插入實驗] 啟動")

while robot.step(TIME_STEP) != -1:
    if state in (COMPLETE, FAILED):
        continue
    watchdog += 1
    plug = plug_node.getPosition()
    port = slot_node.getPosition()

    if state == MOVE_ABOVE:
        control_gripper(0.0)
        pose_target = solve_pose(OBSERVATION_POINTS[scan_index], TOPDOWN_ROTATION)
        apply_pose(pose_target, 0.35)
        state = OBSERVE
        watchdog = 0
        print("1. 移至取料區上方，開啟夾爪相機")

    elif state == OBSERVE:
        if reached("移至視覺觀察位置"):
            observation = see_plug()
            if observation is None:
                vision_wait_steps += 1
                if vision_wait_steps > 20:
                    scan_index += 1
                    vision_wait_steps = 0
                    while scan_index < len(OBSERVATION_POINTS):
                        try:
                            pose_target = solve_pose(
                                OBSERVATION_POINTS[scan_index], TOPDOWN_ROTATION
                            )
                            break
                        except ValueError:
                            scan_index += 1
                    if scan_index == len(OBSERVATION_POINTS):
                        fail("巡看可達區域後，仍未在相機畫面找到 RJ45 插頭")
                    else:
                        apply_pose(pose_target, 0.35)
                        watchdog = 0
                        print(f"[視覺] 巡看第 {scan_index + 1} 個地面區域")
                continue
            detected_plug, detected_yaw = observation
            # 插頭尾端朝向底座時，從插頭另一側夾住護套，避開落地線材。
            reverse_grasp = abs(detected_yaw) > math.radians(120)
            grasp_yaw = (
                detected_yaw - math.copysign(math.pi, detected_yaw)
                if reverse_grasp else detected_yaw
            )
            grasp_rotation = floor_grasp_rotation(grasp_yaw)
            transport_rotation = floor_grasp_rotation(
                math.pi if reverse_grasp else 0.0
            )
            grasp_xy = (
                detected_plug[0] + GRASP_REAR_OFFSET * math.cos(detected_yaw),
                detected_plug[1] + GRASP_REAR_OFFSET * math.sin(detected_yaw),
            )
            print(
                f"[視覺] 插頭位置 X={detected_plug[0]:.3f}, "
                f"Y={detected_plug[1]:.3f}, Z={detected_plug[2]:.3f} m；"
                f"方向={math.degrees(detected_yaw):.1f}°"
            )
            pose_target = solve_pose(
                (grasp_xy[0], grasp_xy[1], FLOOR_GRASP_Z + 0.10),
                grasp_rotation,
            )
            apply_pose(pose_target, 0.15)
            state = DESCEND
            watchdog = 0
            print("1. 依視覺結果移至插頭上方")

    elif state == DESCEND:
        if reached("移至插頭上方"):
            pose_target = solve_pose(
                (grasp_xy[0], grasp_xy[1], FLOOR_GRASP_Z),
                grasp_rotation,
            )
            apply_pose(pose_target, 0.12)
            state = CLOSE
            watchdog = 0
            print("2. 降至插頭後側的夾持位置")

    elif state == CLOSE:
        if reached("下降夾持"):
            control_gripper(0.70)
            previous_gripper = [sensor.getValue() for sensor in gripper_sensors]
            gripper_stable_steps = 0
            state = SETTLE
            watchdog = 0
            print("3. 閉合夾爪")

    elif state == SETTLE:
        positions = [sensor.getValue() for sensor in gripper_sensors]
        stable = all(
            abs(position - previous) < 0.001
            for position, previous in zip(positions, previous_gripper, strict=True)
        )
        gripper_stable_steps = gripper_stable_steps + 1 if stable else 0
        previous_gripper = positions
        if all(abs(position - 0.70) < 0.02 for position in positions):
            fail("夾爪完全閉合，沒有夾到插頭")
        elif gripper_stable_steps >= 10:
            if grasp_latch.getPresence() != 1:
                joints = [sensor.getValue() for sensor in sensors]
                rotation = plug_node.getOrientation()
                actual_grasp = [
                    plug[i] - 0.015 * rotation[3 * i] for i in range(3)
                ]
                print(
                    f"[診斷] 夾爪夾持點={tuple(round(v, 3) for v in grip_point_position(joints))}，"
                    f"插頭護套連接點={tuple(round(v, 3) for v in actual_grasp)}"
                )
                fail("夾爪與插頭護套的夾持點沒有對準")
                continue
            grasp_latch.lock()
            settle_steps = 0
            state = LATCH_SETTLE
            watchdog = 0
            print("3. 夾持護套連接點已鎖定，等待物理穩定")

        elif watchdog > 150:
            fail("夾爪閉合逾時")

    elif state == LATCH_SETTLE:
        settle_steps += 1
        if settle_steps >= 12:
            if not held("抓取確認"):
                continue
            joints = [sensor.getValue() for sensor in sensors]
            transform = forward_kinematics(joints)
            grip = grip_point_position(joints)
            held_offset_local = vector_in_local(
                [row[:3] for row in transform[:3]],
                [plug[i] - grip[i] for i in range(3)],
            )
            move_object((plug[0], plug[1], lift_z), 0.08, grasp_rotation)
            state = LIFT
            print("4. 夾住插頭並離地")

    elif state == LIFT:
        if held("抬升中") and reached("逐步抬升插頭"):
            if abs(plug[2] - lift_z) > OBJECT_TOLERANCE:
                fail("插頭未跟隨夾爪抬升")
            elif lift_z < LIFT_Z - 1e-6:
                lift_z = min(LIFT_Z, lift_z + 0.03)
                move_object((plug[0], plug[1], lift_z), 0.08, grasp_rotation)
            else:
                rotation_center = (
                    plug[0], plug[1], 0.36 if reverse_grasp else LIFT_Z
                )
                move_object(rotation_center, 0.08, grasp_rotation)
                rotation_step = 0
                state = ROTATE_FOR_PORT
                print("5. 抬起後將插頭轉向網路孔")

    elif state == ROTATE_FOR_PORT:
        if held("轉向中") and reached("轉向網路孔"):
            if reverse_grasp and rotation_step < 6:
                rotation_step += 1
                yaw = grasp_yaw + (math.pi - grasp_yaw) * rotation_step / 6
                move_object(
                    rotation_center, 0.06, floor_grasp_rotation(yaw)
                )
                print(f"[轉向] 第 {rotation_step}/6 段")
            else:
                move_object((APPROACH_X, port[1], PORT_Z), 0.10)
                state = MOVE_TO_PORT
                print("5. 將插頭移到網路孔外側")

    elif state == MOVE_TO_PORT:
        if held("搬運中") and reached("移至網路孔"):
            if (
                abs(plug[1] - port[1]) > PORT_ALIGNMENT_TOLERANCE_Y
                or abs(plug[2] - PORT_Z) > PORT_ALIGNMENT_TOLERANCE_Z
            ):
                alignment_attempts += 1
                if alignment_attempts > 5:
                    fail("插頭與網路孔尚未對準")
                else:
                    # 線材重量會讓插頭偏離理想夾持位置；依實測位置重新對位。
                    alignment_bias_y = max(
                        -0.03,
                        min(0.03, alignment_bias_y + port[1] - plug[1]),
                    )
                    alignment_bias_z = max(
                        -0.03,
                        min(0.03, alignment_bias_z + PORT_Z - plug[2]),
                    )
                    joints = [sensor.getValue() for sensor in sensors]
                    transform = forward_kinematics(joints)
                    grip = grip_point_position(joints)
                    held_offset_local = vector_in_local(
                        [row[:3] for row in transform[:3]],
                        [plug[i] - grip[i] for i in range(3)],
                    )
                    move_object(
                        (APPROACH_X, port[1] + alignment_bias_y, PORT_Z + alignment_bias_z),
                        0.06,
                    )
                    print(
                        f"[校正] 網路孔對位第 {alignment_attempts} 次，"
                        f"橫向補償 {alignment_bias_y:.3f} m，"
                        f"高度補償 {alignment_bias_z:.3f} m"
                    )
            else:
                # 對準後重新量測夾持偏移；後續路徑直接使用孔心座標。
                joints = [sensor.getValue() for sensor in sensors]
                transform = forward_kinematics(joints)
                grip = grip_point_position(joints)
                held_offset_local = vector_in_local(
                    [row[:3] for row in transform[:3]],
                    [plug[i] - grip[i] for i in range(3)],
                )
                alignment_bias_y = 0.0
                alignment_bias_z = 0.0
                step_x = APPROACH_X
                move_object(
                    (step_x, port[1] + alignment_bias_y, PORT_Z + alignment_bias_z),
                    0.05,
                )
                state = INSERT
                print("6. 沿 +X 方向插入 RJ45")

    elif state == INSERT:
        plug_at_port = (
            step_x >= INSERT_X - 1e-6
            and distance(plug, (INSERT_X, port[1], PORT_Z))
            <= LATCH_ALIGNMENT_TOLERANCE
        )
        if held("插入中") and (plug_at_port or reached("插入 RJ45")):
            if step_x < INSERT_X - 1e-6:
                step_x = min(INSERT_X, step_x + 0.01)
                move_object(
                    (step_x, port[1] + alignment_bias_y, PORT_Z + alignment_bias_z),
                    0.05,
                )
            elif distance(plug, (INSERT_X, port[1], PORT_Z)) > LATCH_ALIGNMENT_TOLERANCE:
                insertion_corrections += 1
                if insertion_corrections > 8:
                    rotation = plug_node.getOrientation()
                    latch_tip = [
                        plug[i] + 0.025 * rotation[3 * i] for i in range(3)
                    ]
                    print(
                        f"[診斷] 插頭卡扣位置={tuple(round(v, 3) for v in latch_tip)}，"
                        f"孔內卡扣位置={(port[0] - 0.025, port[1], port[2])}"
                    )
                    fail("插頭未到達孔內卡扣位置")
                else:
                    # 關節角到位仍可能留下位置誤差；以插頭實測中心補正。
                    joints = [sensor.getValue() for sensor in sensors]
                    transform = forward_kinematics(joints)
                    grip = grip_point_position(joints)
                    held_offset_local = vector_in_local(
                        [row[:3] for row in transform[:3]],
                        [plug[i] - grip[i] for i in range(3)],
                    )
                    move_object((INSERT_X, port[1], PORT_Z), 0.025)
                    print(f"[校正] 插入終點第 {insertion_corrections} 次")
            else:
                print(
                    f"[檢查] 插入終點 X={plug[0]:.3f}, "
                    f"Y={plug[1]:.3f}, Z={plug[2]:.3f} m"
                )
                grasp_latch.unlock()
                control_gripper(RELEASE_TARGET)
                state = OPEN
                watchdog = 0
                print("7. 插頭到位，於孔外打開夾爪")

    elif state == OPEN:
        # 孔口可能暫時擋住單側夾指；解除夾持後先退一小段，讓夾指有空間繼續張開。
        if (
            all(sensor.getValue() < RELEASE_TARGET + 0.05 for sensor in gripper_sensors)
            or watchdog > 45
        ):
            if watchdog > 45:
                print("[退爪] 單側夾指受阻，先沿孔軸退開")
            step_x = INSERT_X - 0.01
            move_object(
                (step_x, port[1] + alignment_bias_y, PORT_Z + alignment_bias_z),
                0.08,
            )
            state = RETRACT
            print("8. 沿 -X 方向退出夾爪")

    elif state == RETRACT:
        if reached("退爪", 0.055):
            if step_x > APPROACH_X + 1e-6:
                step_x = max(APPROACH_X, step_x - 0.01)
                move_object(
                    (step_x, port[1] + alignment_bias_y, PORT_Z + alignment_bias_z),
                    0.08,
                )
            else:
                control_gripper(0.0)
                placed = plug_node.getPosition()
                print(
                    f"[檢查] 插頭位置 X={placed[0]:.3f}, "
                    f"Y={placed[1]:.3f}, Z={placed[2]:.3f} m"
                )
                if (
                    abs(placed[0] - INSERT_X) > 0.025
                    or abs(placed[1] - port[1]) > PORT_ALIGNMENT_TOLERANCE_Y
                    or abs(placed[2] - PORT_Z) > PORT_ALIGNMENT_TOLERANCE_Z
                ):
                    fail("夾爪退出後，插頭沒有維持在網路孔內")
                else:
                    pose_target = solve_pose(
                        (APPROACH_X + GRASP_REAR_OFFSET, port[1], SAFE_Z),
                        TOPDOWN_ROTATION,
                    )
                    apply_pose(pose_target, 0.15)
                    state = RAISE
                    watchdog = 0
                    print("9. 退到孔外後抬高手臂")

    elif state == RAISE:
        if reached("抬升手臂"):
            placed = plug_node.getPosition()
            if (
                abs(placed[0] - INSERT_X) > 0.025
                or abs(placed[1] - port[1]) > PORT_ALIGNMENT_TOLERANCE_Y
                or abs(placed[2] - PORT_Z) > PORT_ALIGNMENT_TOLERANCE_Z
            ):
                fail("手臂退回後，插頭離開網路孔")
            else:
                print("[完成] RJ45 插頭已放入網路孔，夾爪已退回，線材保持可彎曲。")
                state = COMPLETE
