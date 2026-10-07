"""Port A 單張影像定位；可整檔替換為其他 AOI 或 AI 視覺方法。"""

from collections import deque

import vision_config as config
from vision_geometry import intersect_x_plane, world_ray_from_pixel

# 保留既有匯入介面，供離線測試與其他工具使用。
CAMERA_WIDTH = config.CAMERA_WIDTH
CAMERA_HEIGHT = config.CAMERA_HEIGHT
CAMERA_FOV = config.CAMERA_FOV
AOI = config.AOI
SAMPLE_STEP = config.SAMPLE_STEP
MIN_RIM_CHANNEL = config.MIN_RIM_CHANNEL


def _rim_component(image):
    """找出單一連通且中心中空的外框，避免夾爪亮面干擾。"""
    left, top, right, bottom = AOI
    bright = set()
    for y in range(top, bottom, SAMPLE_STEP):
        for x in range(left, right, SAMPLE_STEP):
            offset = (y * CAMERA_WIDTH + x) * 4
            blue, green, red = image[offset:offset + 3]
            if (min(red, green, blue) > MIN_RIM_CHANNEL
                    and max(red, green, blue) - min(red, green, blue) < config.MAX_RIM_CHROMA):
                bright.add((x, y))

    best = None
    best_score = float("-inf")
    while bright:
        start = bright.pop()
        component = [start]
        queue = deque([start])
        while queue:
            x, y = queue.popleft()
            for dx in (-SAMPLE_STEP, 0, SAMPLE_STEP):
                for dy in (-SAMPLE_STEP, 0, SAMPLE_STEP):
                    neighbor = (x + dx, y + dy)
                    if neighbor in bright:
                        bright.remove(neighbor)
                        component.append(neighbor)
                        queue.append(neighbor)
        if len(component) < config.MIN_COMPONENT_PIXELS:
            continue
        xs = [point[0] for point in component]
        ys = [point[1] for point in component]
        box = (min(xs), min(ys), max(xs), max(ys))
        width = box[2] - box[0]
        height = box[3] - box[1]
        if not (config.RIM_WIDTH_RANGE[0] <= width <= config.RIM_WIDTH_RANGE[1]
                and config.RIM_HEIGHT_RANGE[0] <= height <= config.RIM_HEIGHT_RANGE[1]):
            continue
        center_x = (box[0] + box[2]) / 2
        center_y = (box[1] + box[3]) / 2
        if (abs(center_x - CAMERA_WIDTH / 2) > config.MAX_CENTER_OFFSET[0]
                or abs(center_y - CAMERA_HEIGHT / 2) > config.MAX_CENTER_OFFSET[1]):
            continue
        # 插孔內部應為暗色；填滿的亮色零件不能視為插孔。
        interior = sum(
            abs(x - center_x) < width * config.INTERIOR_HALF_SPAN
            and abs(y - center_y) < height * config.INTERIOR_HALF_SPAN
            for x, y in component
        )
        if interior > len(component) * config.MAX_INTERIOR_RATIO:
            continue
        score = len(component) - config.CENTER_DISTANCE_PENALTY * (
            abs(center_x - CAMERA_WIDTH / 2) + abs(center_y - CAMERA_HEIGHT / 2)
        )
        if score > best_score:
            best = box, len(component)
            best_score = score
    return best


def locate_port(image, camera_position, camera_orientation, port_plane_x):
    """由相機影像找外框，再投影至插孔前平面，回傳 y/z、外框與信心值。

    ``port_plane_x`` 只提供深度；y/z 由實際拍到的影像決定。
    若無可靠外框，回傳 None。
    """
    if not image or len(image) != CAMERA_WIDTH * CAMERA_HEIGHT * 4:
        return None
    rim = _rim_component(image)
    if rim is None:
        return None
    bounding_box, pixel_count = rim
    # 用外框對邊的中點定位，不因局部反光而偏向亮處。
    pixel_x = (bounding_box[0] + bounding_box[2]) / 2
    pixel_y = (bounding_box[1] + bounding_box[3]) / 2
    world_ray = world_ray_from_pixel(
        pixel_x, pixel_y, camera_orientation, CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FOV,
    )
    point = intersect_x_plane(camera_position, world_ray, port_plane_x, config.MIN_FORWARD_RAY_X)
    if point is None:
        return None
    confidence = min(1.0, pixel_count / config.FULL_CONFIDENCE_PIXELS)
    return point[1], point[2], bounding_box, confidence
