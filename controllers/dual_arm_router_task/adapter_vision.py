"""從手臂二相機的橘色外框估計插座開口的三維位置。"""

from collections import deque

import adapter_vision_config as config
from vision_geometry import focal_pixels, world_ray_from_pixel


def _orange_pixels(image):
    """在 AOI 內取出橘色像素，排除黃色清潔頭與灰色 Router。"""
    left, top, right, bottom = config.AOI
    result = set()
    for y in range(top, bottom, config.SAMPLE_STEP):
        for x in range(left, right, config.SAMPLE_STEP):
            offset = (y * config.CAMERA_WIDTH + x) * 4
            blue, green, red = image[offset:offset + 3]
            if (red >= config.MIN_RED and green >= config.MIN_GREEN
                    and red >= green * config.MIN_RED_GREEN_RATIO
                    and green >= blue * config.MIN_GREEN_BLUE_RATIO):
                result.add((x, y))
    return result


def _orange_frame(image):
    """找出連通且中心中空的橘色插座外框。"""
    remaining = _orange_pixels(image)
    candidates = []
    step = config.SAMPLE_STEP
    while remaining:
        first = remaining.pop()
        queue = deque([first])
        points = [first]
        while queue:
            x, y = queue.popleft()
            for dx in (-step, 0, step):
                for dy in (-step, 0, step):
                    neighbor = (x + dx, y + dy)
                    if neighbor in remaining:
                        remaining.remove(neighbor)
                        points.append(neighbor)
                        queue.append(neighbor)
        if len(points) < config.MIN_COMPONENT_PIXELS:
            continue
        xs = [point[0] for point in points]
        ys = [point[1] for point in points]
        box = (min(xs), min(ys), max(xs), max(ys))
        width, height = box[2] - box[0], box[3] - box[1]
        if not (config.WIDTH_RANGE[0] <= width <= config.WIDTH_RANGE[1]
                and config.HEIGHT_RANGE[0] <= height <= config.HEIGHT_RANGE[1]):
            continue
        center_x = (box[0] + box[2]) / 2
        center_y = (box[1] + box[3]) / 2
        interior = sum(
            abs(x - center_x) < width * 0.25 and abs(y - center_y) < height * 0.25
            for x, y in points
        )
        if interior > len(points) * config.MAX_INTERIOR_RATIO:
            continue
        candidates.append((len(points), box))
    return max(candidates, default=None)


def locate_adapter(image, camera_position, camera_orientation):
    """回傳插座 Solid 原點的 (x, y, z)、外框及信心值；未辨識到則回傳 None。

    已知開口外框寬度可由像素寬度估計距離，因此不使用 Supervisor
    取得插座的位置。此方法可換成模型推論，只需維持相同回傳介面。
    """
    if not image or len(image) != config.CAMERA_WIDTH * config.CAMERA_HEIGHT * 4:
        return None
    frame = _orange_frame(image)
    if frame is None:
        return None
    pixel_count, box = frame
    pixel_width = box[2] - box[0]
    depth = focal_pixels(config.CAMERA_WIDTH, config.CAMERA_FOV) * config.FRAME_WIDTH_METERS / pixel_width
    if not config.MIN_CAMERA_DEPTH <= depth <= config.MAX_CAMERA_DEPTH:
        return None
    center_x = (box[0] + box[2]) / 2
    center_y = (box[1] + box[3]) / 2
    ray = world_ray_from_pixel(
        center_x, center_y, camera_orientation,
        config.CAMERA_WIDTH, config.CAMERA_HEIGHT, config.CAMERA_FOV,
    )
    point = tuple(camera_position[axis] + depth * ray[axis] for axis in range(3))
    confidence = min(1.0, pixel_count / config.FULL_CONFIDENCE_PIXELS)
    return point[0] + config.REAR_FACE_TO_ORIGIN_X, point[1], point[2], box, confidence
