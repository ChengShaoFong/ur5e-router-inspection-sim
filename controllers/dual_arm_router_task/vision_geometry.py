"""兩支手臂共用的相機投影計算。"""

import math


def focal_pixels(width, horizontal_fov):
    """將相機水平視角換算為像素焦距。"""
    return width / (2 * math.tan(horizontal_fov / 2))


def world_ray_from_pixel(pixel_x, pixel_y, orientation, width, height, horizontal_fov):
    """將影像像素轉為世界座標射線；Webots 相機局部 +X 朝前。"""
    focal = focal_pixels(width, horizontal_fov)
    camera_ray = (1.0, (width / 2 - pixel_x) / focal, (height / 2 - pixel_y) / focal)
    return tuple(
        sum(orientation[row * 3 + axis] * camera_ray[axis] for axis in range(3))
        for row in range(3)
    )


def intersect_x_plane(position, ray, plane_x, min_forward_x=0.01):
    """計算射線與世界 x 平面的交點；平面在相機後方時回傳 None。"""
    if ray[0] <= min_forward_x:
        return None
    travel = (plane_x - position[0]) / ray[0]
    if travel <= 0:
        return None
    return tuple(position[axis] + travel * ray[axis] for axis in range(3))
