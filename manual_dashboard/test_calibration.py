import json
import math
from pathlib import Path
import tempfile
import unittest

from calibration import CalibrationError, build_reference, map_between_views, save_reference


def example():
    return {
        "freeze_id": "test-freeze",
        "measurements": {"dot_spacing_m": 0.5, "distance_to_a_m": 3.0, "vertical_offset_m": 0.2},
        "views": [
            {"size": [1000, 800], "points": {"a": [100, 200], "b": [300, 200], "pointer": [200, 300]}},
            {"size": [800, 600], "points": {"a": [50, 80], "b": [50, 180], "pointer": [0, 130]}},
        ],
    }


class CalibrationTests(unittest.TestCase):
    def test_two_point_mapping_preserves_rotation_scale_and_translation(self):
        data = example()
        self.assertEqual(map_between_views([200, 300], data["views"][0], data["views"][1]), [0.0, 130.0])

    def test_reference_is_explicitly_wall_only_and_preserves_raw_marks(self):
        data = example()
        result = build_reference(data)
        self.assertEqual(result["status"], "wall_reference_only")
        self.assertFalse(result["axis_calibrated"])
        self.assertEqual(result["views"][0]["points"]["pointer"], [200, 300])
        self.assertAlmostEqual(result["views"][0]["metres_per_pixel_along_ab"], 0.0025)

    def test_rejects_coincident_dots_and_out_of_frame_marks(self):
        for change in (lambda d: d["views"][0]["points"].update(b=[100, 200]),
                       lambda d: d["views"][0]["points"].update(pointer=[1001, 300])):
            data = example()
            change(data)
            with self.assertRaises(CalibrationError):
                build_reference(data)

    def test_rejects_nonfinite_or_missing_measurements(self):
        for value in (float("nan"), float("inf"), -1, None, True):
            data = example()
            data["measurements"]["dot_spacing_m"] = value
            with self.assertRaises(CalibrationError):
                build_reference(data)

    def test_save_preserves_previous_captures_and_exact_source_frames(self):
        with tempfile.TemporaryDirectory() as directory:
            first = save_reference(Path(directory), build_reference(example()), [b"frame-a", b"frame-b"])
            second = save_reference(Path(directory), build_reference(example()), [b"new-a", b"new-b"])
            self.assertNotEqual(first, second)
            self.assertEqual((first / "camera_1.jpg").read_bytes(), b"frame-a")
            self.assertEqual(json.loads((first / "reference.json").read_text())["status"], "wall_reference_only")


if __name__ == "__main__":
    unittest.main()
