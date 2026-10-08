"""讀取精簡的 task.ini，並提供不常調整的演算法預設值。"""

from configparser import ConfigParser
import json
from pathlib import Path
from types import SimpleNamespace


config = ConfigParser(interpolation=None)
path = Path(__file__).with_name("task.ini")
if not config.read(path, encoding="utf-8"):
    raise FileNotFoundError(path)

# 這些是模擬器與視覺演算法的內部預設；平常只需修改 task.ini。
DEFAULTS = {
    "control": dict(
        time_step_ms=32, max_joint_speed=2.5,
        gripper_close_speed=0.5, gripper_open_speed=0.8,
        grip_history_samples=20, max_finger_range=0.015,
        min_contact_angle=0.05, contact_angle_margin=0.03,
        max_grasp_movement=0.012, max_adapter_offset=0.035,
        pickup_check_delay=1.0, pickup_min_height=0.10,
        removal_check_delay=1.0, removal_x_margin=0.04,
        completion_delay=2.0, max_final_joint_error=0.05,
        max_final_floor_error=0.05, floor_target_z=0.021,
    ),
    "port_vision": dict(
        camera_width=320, camera_height=240, camera_fov=0.8,
        aoi=[45, 20, 285, 225], sample_step=2,
        min_rim_channel=45, max_rim_chroma=50, min_component_pixels=65,
        rim_width_range=[20, 115], rim_height_range=[30, 155],
        max_center_offset=[45, 55], max_interior_ratio=0.18,
        interior_half_span=0.25, center_distance_penalty=2,
        full_confidence_pixels=350, min_forward_ray_x=0.01,
        settle_seconds=0.5, camera_period_ms=128, stable_frames=5,
        max_frame_spread=0.0025, min_confidence=0.2,
        max_nominal_error=0.05, max_detection_age=0.5,
        rim_front_x_from_router=-0.114,
    ),
    "adapter_vision": dict(
        camera_width=320, camera_height=240, camera_fov=0.8,
        camera_period_ms=128, aoi=[25, 15, 295, 230],
        camera_tool_translation=[-0.04, 0.0, 0.07],
        camera_tool_y_rotation=-1.25, sample_step=2,
        min_red=35, min_green=15, min_red_green_ratio=1.4,
        min_green_blue_ratio=1.2, min_component_pixels=35,
        width_range=[20, 240], height_range=[18, 190],
        max_interior_ratio=0.2, full_confidence_pixels=220,
        frame_width_meters=0.05, rear_face_to_origin_x=0.051,
        min_camera_depth=0.05, max_camera_depth=0.8,
        settle_seconds=0.5, stable_frames=3,
        max_frame_spread=0.012, max_detection_age=4.0,
        min_confidence=0.2, max_nominal_x_error=0.12,
        max_nominal_y_error=0.12, max_nominal_z_error=0.10,
        min_replan_shift=0.003, min_replan_interval=0.4,
    ),
}


def section(name):
    """以內部預設為基礎，套用 INI 中實際寫出的設定。"""
    values = DEFAULTS.get(name, {}).copy()
    if config.has_section(name):
        values.update({key: json.loads(value) for key, value in config.items(name)})
    return SimpleNamespace(**{key.upper(): value for key, value in values.items()})


task = section("task")
calibration = section("calibration")
control = section("control")
port_vision = section("port_vision")
adapter_vision = section("adapter_vision")
port_vision.SAMPLE_INTERVAL = port_vision.CAMERA_PERIOD_MS / 1000
adapter_vision.SAMPLE_INTERVAL = adapter_vision.CAMERA_PERIOD_MS / 1000
