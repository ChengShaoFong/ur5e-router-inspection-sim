"""Image-only localization of the bright rim around router port A."""

import math
from collections import deque


CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240
CAMERA_FOV = 0.8
AOI = (85, 60, 215, 175)
SAMPLE_STEP = 2
MIN_RIM_CHANNEL = 45


def _rim_component(image):
    """Find one connected, hollow port rim instead of averaging all bright objects."""
    left, top, right, bottom = AOI
    bright = set()
    for y in range(top, bottom, SAMPLE_STEP):
        for x in range(left, right, SAMPLE_STEP):
            offset = (y * CAMERA_WIDTH + x) * 4
            blue, green, red = image[offset:offset + 3]
            if min(red, green, blue) > MIN_RIM_CHANNEL and max(red, green, blue) - min(red, green, blue) < 50:
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
        if len(component) < 65:
            continue
        xs = [point[0] for point in component]
        ys = [point[1] for point in component]
        box = (min(xs), min(ys), max(xs), max(ys))
        width = box[2] - box[0]
        height = box[3] - box[1]
        if not (20 <= width <= 115 and 30 <= height <= 155):
            continue
        center_x = (box[0] + box[2]) / 2
        center_y = (box[1] + box[3]) / 2
        if abs(center_x - CAMERA_WIDTH / 2) > 45 or abs(center_y - CAMERA_HEIGHT / 2) > 55:
            continue
        # The router opening is dark. A filled, bright gripper part is not a port.
        interior = sum(
            abs(x - center_x) < width * 0.25 and abs(y - center_y) < height * 0.25
            for x, y in component
        )
        if interior > len(component) * 0.18:
            continue
        score = len(component) - 2 * (abs(center_x - CAMERA_WIDTH / 2) + abs(center_y - CAMERA_HEIGHT / 2))
        if score > best_score:
            best = box, len(component)
            best_score = score
    return best


def locate_port(image, camera_position, camera_orientation, port_plane_x):
    """Return (world_y, world_z, bounding_box, confidence) or None.

    Detect the bright port rim in Camera pixels, then intersect its viewing ray
    with the router's front plane. The plane supplies depth only; y/z come from
    the moving arm camera image.
    """
    if not image or len(image) != CAMERA_WIDTH * CAMERA_HEIGHT * 4:
        return None
    rim = _rim_component(image)
    if rim is None:
        return None
    bounding_box, pixel_count = rim
    # Opposite sides of the rectangular rim define its center even when their
    # brightness differs. The mean of bright pixels drifts toward highlights.
    pixel_x = (bounding_box[0] + bounding_box[2]) / 2
    pixel_y = (bounding_box[1] + bounding_box[3]) / 2
    focal = CAMERA_WIDTH / (2 * math.tan(CAMERA_FOV / 2))
    # Webots Camera looks along local +X, with +Y left and +Z up.
    camera_ray = (1.0, (CAMERA_WIDTH / 2 - pixel_x) / focal,
                  (CAMERA_HEIGHT / 2 - pixel_y) / focal)
    world_ray = tuple(
        sum(camera_orientation[row * 3 + axis] * camera_ray[axis] for axis in range(3))
        for row in range(3)
    )
    if world_ray[0] <= 0.01:
        return None
    travel = (port_plane_x - camera_position[0]) / world_ray[0]
    if travel <= 0:
        return None
    world_y = camera_position[1] + travel * world_ray[1]
    world_z = camera_position[2] + travel * world_ray[2]
    confidence = min(1.0, pixel_count / 350)
    return world_y, world_z, bounding_box, confidence
