"""UR5e 末端正向與逆向運動學；與任務時程、視覺辨識分離。"""

import math


JOINT_NAMES = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)

# 關節軸與連桿座標依 Webots R2025a 的 UR5e.proto。
LINKS = (
    ((0, 0, 1), (0, 0, 0.163), (0, 0, 0.163), (0, 0, 1, 0)),
    ((0, 1, 0), (0, 0.138, 0), (0, 0.138, 0), (0, 1, 0, math.pi / 2)),
    ((0, 1, 0), (0, -0.131, 0.425), (0, -0.131, 0.425), (0, 1, 0, 0)),
    ((0, 1, 0), (0, 0, 0.392), (0, 0, 0.392), (0, 1, 0, math.pi / 2)),
    ((0, 0, 1), (0, 0.127, 0), (0, 0.127, 0), (0, 0, 1, 0)),
    ((0, 1, 0), (0, 0, 0.1), (0, 0, 0.1), (0, 1, 0, 0)),
)
TOOL_SLOT_OFFSET = (0, 0.1, 0)
TIP_OFFSET = (0, 0, 0.20)
LIMITS = ((-2 * math.pi, 2 * math.pi),) * 2 + ((-math.pi, math.pi),) + ((-2 * math.pi, 2 * math.pi),) * 3

TOP_DOWN = ((1, 0, 0), (0, -1, 0), (0, 0, -1))
# 工具局部 +Z 指向世界 +X，也就是插入方向。
HORIZONTAL = ((0, 0, 1), (0, 1, 0), (-1, 0, 0))
HORIZONTAL_ROLLED = ((0, 0, 1), (0, -1, 0), (1, 0, 0))


def horizontal_roll(angle):
    """保持工具 +Z 沿世界 +X，並繞插入軸旋轉指定角度。"""
    sine, cosine = math.sin(angle), math.cos(angle)
    return ((0, 0, 1), (sine, cosine, 0), (-cosine, sine, 0))


def interpolate_orientation(start, end, fraction):
    """按 fraction 在兩個世界座標姿態間內插。"""
    vector = orientation_error(end, start)
    angle = math.sqrt(sum(value * value for value in vector))
    if angle < 1e-9:
        return start
    delta = rotation(tuple(value / angle for value in vector), angle * fraction)
    return tuple(
        tuple(sum(delta[row][axis] * start[axis][column] for axis in range(3)) for column in range(3))
        for row in range(3)
    )


def multiply(a, b):
    """相乘兩個 4×4 齊次轉換矩陣。"""
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def translation(offset):
    """建立指定 x/y/z 位移的齊次轉換矩陣。"""
    return [[1, 0, 0, offset[0]], [0, 1, 0, offset[1]], [0, 0, 1, offset[2]], [0, 0, 0, 1]]


def rotation(axis, angle):
    """建立繞指定軸旋轉 angle 弧度的矩陣。"""
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    v = 1 - c
    return [
        [x * x * v + c, x * y * v - z * s, x * z * v + y * s, 0],
        [y * x * v + z * s, y * y * v + c, y * z * v - x * s, 0],
        [z * x * v - y * s, z * y * v + x * s, z * z * v + c, 0],
        [0, 0, 0, 1],
    ]


def forward_kinematics(joints):
    """由六個關節角求出工具安裝座在手臂底座座標中的姿態。"""
    frame = translation((0, 0, 0))
    for angle, (axis, anchor, endpoint, fixed) in zip(joints, LINKS):
        frame = multiply(frame, translation(anchor))
        frame = multiply(frame, rotation(axis, angle))
        frame = multiply(frame, translation(tuple(-v for v in anchor)))
        frame = multiply(frame, translation(endpoint))
        frame = multiply(frame, rotation(fixed[:3], fixed[3]))
    return multiply(frame, translation(TOOL_SLOT_OFFSET))


def orientation_error(target, current):
    """以旋轉向量表示目標姿態相對目前姿態的誤差。"""
    relative = [[sum(target[i][k] * current[j][k] for k in range(3)) for j in range(3)] for i in range(3)]
    skew = [
        (relative[2][1] - relative[1][2]) / 2,
        (relative[0][2] - relative[2][0]) / 2,
        (relative[1][0] - relative[0][1]) / 2,
    ]
    sine = math.sqrt(sum(value * value for value in skew))
    cosine = max(-1.0, min(1.0, (sum(relative[i][i] for i in range(3)) - 1) / 2))
    angle = math.atan2(sine, cosine)
    if sine > 1e-8:
        return [value * angle / sine for value in skew]
    if cosine > 0:
        return [0.0, 0.0, 0.0]
    # 轉角為 180 度時反對稱部分消失，從對角線恢復旋轉軸。
    axis = [math.sqrt(max(0.0, (relative[i][i] + 1) / 2)) for i in range(3)]
    largest = max(range(3), key=lambda i: axis[i])
    for i in range(3):
        if i != largest and relative[largest][i] + relative[i][largest] < 0:
            axis[i] = -axis[i]
    return [value * math.pi for value in axis]


def solve_linear(matrix, vector):
    """用帶選主元的消去法解線性方程組。"""
    count = len(vector)
    rows = [matrix[i][:] + [vector[i]] for i in range(count)]
    for column in range(count):
        pivot = max(range(column, count), key=lambda row: abs(rows[row][column]))
        rows[column], rows[pivot] = rows[pivot], rows[column]
        if abs(rows[column][column]) < 1e-12:
            raise ValueError("Singular IK matrix")
        scale = rows[column][column]
        rows[column] = [value / scale for value in rows[column]]
        for row in range(count):
            if row != column:
                factor = rows[row][column]
                rows[row] = [rows[row][j] - factor * rows[column][j] for j in range(count + 1)]
    return [rows[i][-1] for i in range(count)]


def solve_pose(world_tip, orientation, base, seed=None):
    """依世界座標工具點與姿態求 UR5e 關節角；底座沒有額外偏航。"""
    target = [
        world_tip[i] - base[i] - sum(orientation[i][k] * TIP_OFFSET[k] for k in range(3))
        for i in range(3)
    ]
    joints = list(seed if seed is not None else (0, -1.0, 1.8, -1.0, 0, 0))
    epsilon = 1e-5
    for _ in range(180):
        frame = forward_kinematics(joints)
        position_error = [target[i] - frame[i][3] for i in range(3)]
        angle_error = orientation_error(orientation, [row[:3] for row in frame[:3]])
        if math.dist(target, [frame[i][3] for i in range(3)]) < 0.004 and math.sqrt(sum(x*x for x in angle_error)) < 0.035:
            return tuple(joints)
        error = position_error + angle_error
        jacobian = [[0.0] * 6 for _ in range(6)]
        for column in range(6):
            perturbed = joints[:]
            perturbed[column] += epsilon
            next_frame = forward_kinematics(perturbed)
            delta_rotation = orientation_error([row[:3] for row in next_frame[:3]], [row[:3] for row in frame[:3]])
            for row in range(3):
                jacobian[row][column] = (next_frame[row][3] - frame[row][3]) / epsilon
                jacobian[row + 3][column] = delta_rotation[row] / epsilon
        normal = [[sum(jacobian[k][i] * jacobian[k][j] for k in range(6)) + (0.015 if i == j else 0) for j in range(6)] for i in range(6)]
        projected = [sum(jacobian[k][i] * error[k] for k in range(6)) for i in range(6)]
        change = solve_linear(normal, projected)
        factor = min(1.0, 0.20 / max(abs(v) for v in change))
        joints = [max(LIMITS[i][0], min(LIMITS[i][1], joints[i] + change[i] * factor)) for i in range(6)]
    raise ValueError(f"No IK solution for tip {world_tip} from base {base}")
