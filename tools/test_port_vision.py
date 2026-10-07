"""Synthetic image checks for hand-camera port localization."""

import math
import sys
import unittest
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "controllers" / "dual_arm_router_task"))
from port_vision import CAMERA_HEIGHT, CAMERA_WIDTH, CAMERA_FOV, locate_port  # noqa: E402


def image_with_box():
    image = bytearray(CAMERA_WIDTH * CAMERA_HEIGHT * 4)

    def rectangle(left, top, right, bottom, brightness):
        for y in range(top, bottom):
            for x in range(left, right):
                offset = (y * CAMERA_WIDTH + x) * 4
                image[offset:offset + 4] = bytes((brightness,) * 3 + (255,))

    return image, rectangle


class PortVisionTest(unittest.TestCase):
    def test_rim_center_ignores_uneven_lighting_and_other_bright_parts(self):
        image, rectangle = image_with_box()
        # Frame center is (174, 108); its lower rail has fewer bright pixels.
        rectangle(130, 69, 219, 76, 200)
        rectangle(130, 141, 176, 148, 160)
        rectangle(130, 69, 137, 148, 190)
        rectangle(212, 69, 219, 148, 210)
        rectangle(115, 43, 186, 54, 200)  # Bright bar above the port in Webots.
        rectangle(97, 165, 124, 194, 180)  # Unrelated bright robot surface.
        detection = locate_port(bytes(image), (0.45, 0.08, 0.30),
                                (1, 0, 0, 0, 1, 0, 0, 0, 1), 0.656)
        self.assertIsNotNone(detection)
        world_y, world_z, box, _ = detection
        focal = CAMERA_WIDTH / (2 * math.tan(CAMERA_FOV / 2))
        self.assertLess(abs(world_y - (0.08 + 0.206 * (160 - 174) / focal)), 0.0015)
        self.assertLess(abs(world_z - (0.30 + 0.206 * (120 - 108) / focal)), 0.0015)
        self.assertLess(abs((box[0] + box[2]) / 2 - 174), 3)
        self.assertLess(abs((box[1] + box[3]) / 2 - 108), 3)

    def test_filled_bright_part_is_not_a_port(self):
        image, rectangle = image_with_box()
        rectangle(130, 70, 211, 151, 200)
        self.assertIsNone(locate_port(bytes(image), (0.45, 0.08, 0.30),
                                      (1, 0, 0, 0, 1, 0, 0, 0, 1), 0.656))

    def test_dim_webots_rim_is_still_localized(self):
        image, rectangle = image_with_box()
        rectangle(110, 78, 210, 86, 52)
        rectangle(110, 148, 210, 161, 55)
        rectangle(110, 78, 121, 161, 51)
        rectangle(199, 78, 210, 161, 52)
        detection = locate_port(bytes(image), (0.45, 0.08, 0.30),
                                (1, 0, 0, 0, 1, 0, 0, 0, 1), 0.656)
        self.assertIsNotNone(detection)
        self.assertLess(abs((detection[2][0] + detection[2][2]) / 2 - 160), 2)
        self.assertLess(abs((detection[2][1] + detection[2][3]) / 2 - 119), 2)


if __name__ == "__main__":
    unittest.main()
